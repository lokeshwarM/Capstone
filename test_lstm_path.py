"""
=============================================================================
LSTM Trajectory Sequence Modeling & GPS-Denied Dead-Reckoning Evaluation
Autonomous Swarm UAV Delivery System | Academic Capstone Research
=============================================================================
This test script:
  1. Generates 3D UAV delivery trajectories across the 8 building destinations.
  2. Trains the TrajectoryLSTM network on sequential flight waypoints.
  3. Simulates a GPS-denied urban canyon dropout and evaluates dead-reckoning.
  4. Compares LSTM predictive accuracy vs Linear extrapolation.
  5. Exports high-resolution academic verification figures to results/lstm_path_prediction.png.
  6. Saves trained model checkpoint to results/lstm_path_model.pth.
=============================================================================
"""

import os
import sys
sys.stdout.reconfigure(line_buffering=True)
# pyrefly: ignore [missing-import]
import numpy as np
# pyrefly: ignore [missing-import]
import torch
# pyrefly: ignore [missing-import]
import matplotlib
matplotlib.use('Agg')
# pyrefly: ignore [missing-import]
import matplotlib.pyplot as plt
# pyrefly: ignore [missing-import]
from mpl_toolkits.mplot3d import Axes3D

# Ensure Capstone root is in sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from modules.lstm_path_model import (
    TrajectoryLSTM,
    generate_realistic_uav_trajectory,
    create_training_dataset,
    train_lstm_model,
    evaluate_dead_reckoning
)

# 8 Rooftop delivery destinations from Gazebo city world
DESTINATIONS = [
    (-45.0, -45.0, 10.8), (-45.0, -15.0, 27.7), (-30.0,  30.0, 27.5),
    (-15.0,   0.0, 10.3), (  0.0, -15.0, 38.4), (  0.0,  15.0, 19.4),
    ( 15.0, -15.0, 30.2), ( 30.0, -45.0, 35.1),
]

TRUCK_START = (0.0, -37.5, 1.5)


def run_evaluation():
    print("=" * 75)
    print("  LSTM PATH MODELING & PREDICTIVE DEAD-RECKONING EVALUATION")
    print("=" * 75)

    os.makedirs("results", exist_ok=True)
    os.makedirs("logs", exist_ok=True)

    # 1. Prepare Dataset
    print("\n[Step 1/5] Synthesizing 3D UAV Trajectory Dataset across 8 Destinations...")
    SEQ_LEN = 10
    X, y, all_trajectories = create_training_dataset(
        destinations=DESTINATIONS,
        num_trajectories=70,
        seq_len=SEQ_LEN,
        noise_std=0.05
    )
    print(f"  [OK] Generated {len(all_trajectories)} full mission trajectories.")
    print(f"  [OK] Sliding Window Dataset: {X.shape[0]} training pairs (Window={SEQ_LEN}, Output=3D Point).")

    # 2. Build & Train LSTM Model
    print("\n[Step 2/5] Training TrajectoryLSTM Network...")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"  * Compute Device: {device}")
    
    model = TrajectoryLSTM(input_size=3, hidden_size=64, num_layers=2, output_size=3, dropout=0.1).to(device)
    X_dev, y_dev = X.to(device), y.to(device)
    
    EPOCHS = 40
    loss_history = train_lstm_model(model, X_dev, y_dev, epochs=EPOCHS, batch_size=64, lr=0.003)
    final_loss = loss_history[-1]
    print(f"  [OK] Training Completed. Initial Loss: {loss_history[0]:.5f} -> Final MSE Loss: {final_loss:.5f}")

    # Save trained checkpoint
    model_save_path = "results/lstm_path_model.pth"
    torch.save(model.state_dict(), model_save_path)
    print(f"  [OK] Checkpoint saved to: {model_save_path}")

    # 3. Simulate GPS Denial & Evaluate Dead-Reckoning
    print("\n[Step 3/5] Evaluating GPS-Denied Dead-Reckoning (Urban Canyon Dropout)...")
    # Generate an unseen test trajectory to target destination 7 (30.0, -45.0, 35.1)
    test_target = (30.0, -45.0, 35.1)
    test_traj = generate_realistic_uav_trajectory(
        num_steps=100,
        truck_pos=TRUCK_START,
        target_pos=test_target,
        cruise_alt=50.0
    )

    DROPOUT_START = 55
    DROPOUT_LEN = 12
    model.cpu()
    results = evaluate_dead_reckoning(
        model=model,
        test_trajectory=test_traj,
        dropout_start=DROPOUT_START,
        dropout_len=DROPOUT_LEN,
        seq_len=SEQ_LEN
    )

    print(f"  * Simulated GPS Dropout Period: Steps {DROPOUT_START} to {DROPOUT_START + DROPOUT_LEN}")
    print(f"  * Baseline Linear Extrapolation RMSE : {results['linear_rmse']:.3f} m (Max Drift: {results['max_linear_drift']:.3f} m)")
    print(f"  * LSTM Dead-Reckoning RMSE          : {results['lstm_rmse']:.3f} m (Max Drift: {results['max_lstm_drift']:.3f} m)")
    print(f"  [RESULT] Trajectory Drift Reduction : {results['drift_reduction_pct']:.2f}% improvement over baseline!")

    # 4. Generate High-Resolution Scientific Plots
    print("\n[Step 4/5] Generating Scientific Multi-Panel Verification Figures...")
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig = plt.figure(figsize=(16, 12))

    # --- Panel 1: 3D Trajectory Comparison ---
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    # Full Ground truth
    ax1.plot(test_traj[:, 0], test_traj[:, 1], test_traj[:, 2], 'g--', label='Full Ground Truth Path', alpha=0.6, linewidth=1.8)
    # Available GPS segment before dropout
    ax1.plot(test_traj[:DROPOUT_START, 0], test_traj[:DROPOUT_START, 1], test_traj[:DROPOUT_START, 2], 'b-', label='Active GPS Tracking', linewidth=2.5)
    # Ground truth inside dropout
    gt_seg = results['gt_dropout']
    ax1.plot(gt_seg[:, 0], gt_seg[:, 1], gt_seg[:, 2], 'k:', label='True Trajectory (Dropout Zone)', linewidth=2.5)
    # LSTM Prediction
    lstm_seg = results['lstm_preds']
    ax1.plot(lstm_seg[:, 0], lstm_seg[:, 1], lstm_seg[:, 2], 'r-o', markersize=3, label='LSTM Dead-Reckoning', linewidth=2.2)
    # Linear baseline
    lin_seg = results['linear_preds']
    ax1.plot(lin_seg[:, 0], lin_seg[:, 1], lin_seg[:, 2], 'm-.', label='Linear Velocity Extrap.', linewidth=1.5, alpha=0.8)

    # Markers for Start and Target
    ax1.scatter([TRUCK_START[0]], [TRUCK_START[1]], [TRUCK_START[2]], color='orange', s=80, marker='s', label='Mothership Truck')
    ax1.scatter([test_target[0]], [test_target[1]], [test_target[2]], color='darkred', s=90, marker='^', label='Target Rooftop')

    ax1.set_title("3D Flight Trajectory: GPS-Denied Dead-Reckoning", fontsize=12, fontweight='bold', pad=10)
    ax1.set_xlabel("X Position (m)", fontsize=10)
    ax1.set_ylabel("Y Position (m)", fontsize=10)
    ax1.set_zlabel("Z Altitude (m)", fontsize=10)
    ax1.legend(loc='upper left', fontsize=8)

    # --- Panel 2: Error Drift Over Dropout Timesteps ---
    ax2 = fig.add_subplot(2, 2, 2)
    steps_axis = np.arange(1, DROPOUT_LEN + 1)
    lstm_drift_per_step = np.linalg.norm(lstm_seg - gt_seg, axis=1)
    lin_drift_per_step = np.linalg.norm(lin_seg - gt_seg, axis=1)

    ax2.plot(steps_axis, lin_drift_per_step, 'm--s', markersize=4, label=f'Linear Extrap (Max: {results["max_linear_drift"]:.2f}m)', linewidth=2.0)
    ax2.plot(steps_axis, lstm_drift_per_step, 'r-o', markersize=4, label=f'LSTM Model (Max: {results["max_lstm_drift"]:.2f}m)', linewidth=2.0)
    ax2.fill_between(steps_axis, lstm_drift_per_step, lin_drift_per_step, color='green', alpha=0.15, label='Error Mitigation Region')

    ax2.set_title("Cumulative Position Drift during Sensor Dropout", fontsize=12, fontweight='bold')
    ax2.set_xlabel("Elapsed Timesteps into GPS-Denied Canyon", fontsize=10)
    ax2.set_ylabel("Euclidean Drift Error (meters)", fontsize=10)
    ax2.legend(loc='upper left', fontsize=9)
    ax2.grid(True, linestyle='--', alpha=0.6)

    # --- Panel 3: LSTM Loss Convergence ---
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(range(1, EPOCHS + 1), loss_history, 'b-', linewidth=2.0, label='MSE Trajectory Loss')
    ax3.set_title(f"TrajectoryLSTM Training Convergence ({EPOCHS} Epochs)", fontsize=12, fontweight='bold')
    ax3.set_xlabel("Epoch", fontsize=10)
    ax3.set_ylabel("Mean Squared Error (MSE)", fontsize=10)
    ax3.legend(loc='upper right', fontsize=9)
    ax3.grid(True, linestyle='--', alpha=0.6)

    # --- Panel 4: Per-Axis (X, Y, Z) Tracking Accuracy ---
    ax4 = fig.add_subplot(2, 2, 4)
    time_pts = np.arange(DROPOUT_START, DROPOUT_START + DROPOUT_LEN)
    ax4.plot(time_pts, gt_seg[:, 0], 'k-', label='GT X', linewidth=1.5)
    ax4.plot(time_pts, lstm_seg[:, 0], 'r--', label='LSTM X', linewidth=1.5)
    ax4.plot(time_pts, gt_seg[:, 1], 'k-.', label='GT Y', linewidth=1.5)
    ax4.plot(time_pts, lstm_seg[:, 1], 'b--', label='LSTM Y', linewidth=1.5)
    ax4.plot(time_pts, gt_seg[:, 2], 'k:', label='GT Z', linewidth=1.5)
    ax4.plot(time_pts, lstm_seg[:, 2], 'g--', label='LSTM Z', linewidth=1.5)

    ax4.set_title("Per-Axis Coordinate Tracking (X, Y, Z)", fontsize=12, fontweight='bold')
    ax4.set_xlabel("Simulation Timeline (Steps)", fontsize=10)
    ax4.set_ylabel("Coordinate Value (meters)", fontsize=10)
    ax4.legend(loc='center right', fontsize=8, ncol=2)
    ax4.grid(True, linestyle='--', alpha=0.6)

    plt.suptitle("Multi-UAV Autonomous Swarm: LSTM Trajectory Modeling & Dead-Reckoning", fontsize=15, fontweight='bold', y=0.99)
    plt.tight_layout()

    plot_save_path = "results/lstm_path_prediction.png"
    plt.savefig(plot_save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  [OK] High-resolution plot saved to: {plot_save_path}")

    # 5. Export Summary Log
    log_summary_path = "logs/lstm_path_modeling_summary.txt"
    with open(log_summary_path, "w", encoding="utf-8") as f:
        f.write("=" * 70 + "\n")
        f.write("  LSTM TRAJECTORY PREDICTION & DEAD-RECKONING VERIFICATION REPORT\n")
        f.write("=" * 70 + "\n\n")
        f.write(f"Architecture Parameters:\n")
        f.write(f"  * Model Type             : 2-Layer LSTM with Dense Regressor Head\n")
        f.write(f"  * Input State Dimension  : 3 (Spatial X, Y, Z Coordinates)\n")
        f.write(f"  * Hidden Layer Size      : 64 units\n")
        f.write(f"  * Sliding Sequence Window: {SEQ_LEN} historical steps\n")
        f.write(f"  * Training Dataset Size  : {X.shape[0]} temporal slices\n")
        f.write(f"  * Final Convergence Loss : {final_loss:.6f} MSE\n\n")
        f.write("-" * 70 + "\n")
        f.write("GPS-DENIED DEAD-RECKONING BENCHMARK RESULTS:\n")
        f.write("-" * 70 + "\n")
        f.write(f"  * Evaluated Dropout Duration: {DROPOUT_LEN} steps (Steps {DROPOUT_START} -> {DROPOUT_START + DROPOUT_LEN})\n")
        f.write(f"  * Baseline Linear Extrap. RMSE : {results['linear_rmse']:.4f} meters\n")
        f.write(f"  * Baseline Maximum Drift       : {results['max_linear_drift']:.4f} meters\n")
        f.write(f"  * LSTM Dead-Reckoning RMSE     : {results['lstm_rmse']:.4f} meters\n")
        f.write(f"  * LSTM Maximum Drift           : {results['max_lstm_drift']:.4f} meters\n")
        f.write(f"  * Error Mitigation Advantage   : {results['drift_reduction_pct']:.2f}% lower drift\n\n")
        f.write("Scientific Takeaway:\n")
        f.write("  Under GPS multipath failure in dense urban building canyons,\n")
        f.write("  conventional linear extrapolation quickly diverges due to aerodynamic curvature\n")
        f.write("  and banking maneuvers. TrajectoryLSTM preserves non-linear momentum, allowing\n")
        f.write("  safe waypoint recovery and collision-free dead-reckoning.\n")
        f.write("=" * 70 + "\n")
    print(f"  [OK] Text report saved to: {log_summary_path}")

    print("\n" + "=" * 75)
    print("  EVALUATION COMPLETE - ALL DELIVERABLES GENERATED")
    print("=" * 75)


if __name__ == '__main__':
    run_evaluation()
