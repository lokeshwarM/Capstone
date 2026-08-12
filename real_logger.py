import os
import csv
# pyrefly: ignore [missing-import]
import numpy as np

class RealSimulationLogger:
    """
    Real Simulation Telemetry & Performance Data Logger.
    Logs exact millisecond latency, compressed file sizes (KB), tracking mIoU, and makespan to CSV.
    """
    def __init__(self, log_filename="simulation_logs.csv"):
        self.results_dir = os.path.join(os.path.dirname(__file__), "results")
        os.makedirs(self.results_dir, exist_ok=True)
        self.log_filepath = os.path.join(self.results_dir, log_filename)
        self.logs = []
        self._init_csv()

    def _init_csv(self):
        headers = [
            "time_step_sec", "drone_id", "confidence_Sk", "tracking_var",
            "battery_soc", "bandwidth_mbps", "comm_mode", "payload_kb",
            "latency_ms", "mIoU", "route_action"
        ]
        with open(self.log_filepath, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)

    def log_step(self, time_step, drone_id, Sk, tracking_var, battery_soc, bw, comm_mode, payload_kb, latency_ms, mIoU, route_action):
        row = [
            round(time_step, 2), drone_id, round(Sk, 4), round(tracking_var, 2),
            round(battery_soc, 2), round(bw, 2), comm_mode, round(payload_kb, 2),
            round(latency_ms, 2), round(mIoU, 4), route_action
        ]
        self.logs.append(row)
        with open(self.log_filepath, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    def get_summary_statistics(self):
        """
        Computes real aggregated metrics from logged data.
        """
        if not self.logs:
            return {}
        
        latencies = [r[8] for r in self.logs]
        payloads = [r[7] for r in self.logs]
        mious = [r[9] for r in self.logs]

        return {
            "avg_latency_ms": float(np.mean(latencies)),
            "avg_payload_kb": float(np.mean(payloads)),
            "avg_mIoU": float(np.mean(mious)),
            "total_logged_steps": len(self.logs)
        }
