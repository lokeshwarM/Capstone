import os
import sys
import time

# Ensure current directory is in python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from modules.decision_engine import AIDecisionEngine
from benchmark import run_benchmarks

def main():
    print("===================================================================")
    print("  ADAPTIVE AI DECISION ENGINE - CAPSTONE PROJECT SIMULATOR")
    print("  VIT-AP University | Scope | Multi-UAV Dynamic Coordination")
    print("===================================================================")

    # 1. Initialize Engine
    print("\n[Init] Initializing Event-Driven AI Decision Engine...")
    engine = AIDecisionEngine(
        conf_threshold=0.65,
        drift_threshold=15.0,
        battery_threshold=20.0,
        bw_threshold=5.0
    )
    print("[Init] Modules Loaded: YOLOv8 Perception, LK Optical Flow, ACO Allocator, ALNS Router.")

    # 2. Run Telemetry Scenarios
    print("\n[Execution] Running Telemetry Evaluation Steps:")
    sample_scenarios = [
        {
            "name": "Scenario 1: High Confidence, Low Drift, Full Battery",
            "telemetry": {"confidence_Sk": 0.88, "drift_variance": 4.2, "battery_soc": 85.0, "bandwidth_bk": 8.0, "drone_id": 1}
        },
        {
            "name": "Scenario 2: Low Confidence, High Drift (Camera Tilt Trigger)",
            "telemetry": {"confidence_Sk": 0.45, "drift_variance": 22.5, "battery_soc": 60.0, "bandwidth_bk": 4.0, "drone_id": 2}
        },
        {
            "name": "Scenario 3: Low Battery Trigger (ALNS Re-route)",
            "telemetry": {"confidence_Sk": 0.75, "drift_variance": 6.1, "battery_soc": 14.5, "bandwidth_bk": 6.0, "drone_id": 3}
        }
    ]

    for sc in sample_scenarios:
        print(f"\n---> {sc['name']}")
        telemetry = sc['telemetry']
        print(f"     State: Sk={telemetry['confidence_Sk']} | Drift={telemetry['drift_variance']} | SoC={telemetry['battery_soc']}% | BW={telemetry['bandwidth_bk']}Mbps")
        
        actions = engine.evaluate_state_vector(telemetry)
        print(f"     Decision Output:")
        print(f"       • Comm Mode   : {actions['comm_mode']} (Payload: {actions['payload_kb']} KB)")
        print(f"       • Drift Status: {actions['drift_status']} (Peer Correction: {actions['peer_correction_triggered']})")
        print(f"       • Route Action: {actions['route_action']}")

    # 3. ACO Initial Allocation Demo
    print("\n[Execution] Testing ACO Task Allocation Engine (Paper 1)...")
    drone_positions = [(0, 0), (100, 0), (0, 100), (100, 100)]
    target_positions = [(20, 30), (50, 80), (70, 10), (90, 90), (10, 60), (40, 40), (80, 20), (30, 90)]
    assignments, total_dist = engine.aco_allocator.solve(drone_positions, target_positions, iterations=15)
    print(f"     ACO Task Assignment Complete:")
    for drone_id, targets in assignments.items():
        print(f"       • Drone {drone_id} assigned targets: {targets}")
    print(f"     Total Initial ACO Route Distance: {total_dist:.2f} meters")

    # 4. Generate Benchmark Figures for Paper
    print("\n[Execution] Generating Benchmark Plots for IEEE Paper...")
    run_benchmarks()

    print("\n===================================================================")
    print("  [COMPLETE] All System Modules Executed Cleanly!")
    print("===================================================================")

if __name__ == "__main__":
    main()
