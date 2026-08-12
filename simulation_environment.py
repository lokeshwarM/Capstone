import math
import numpy as np

class DroneAgent:
    """
    Real Drone Physics Agent.
    Tracks 2D Position, Velocity, Battery State-of-Charge (SoC), Payload, and Active Route.
    """
    def __init__(self, drone_id, start_pos=(0.0, 0.0), max_speed=15.0, battery_soc=100.0, max_payload_kg=3.0):
        self.drone_id = drone_id
        self.pos = np.array(start_pos, dtype=np.float64)
        self.max_speed = max_speed        # meters per second
        self.battery_soc = battery_soc    # percentage [0, 100]
        self.current_payload_kg = 1.0     # current carried package/camera payload
        self.active_route = []            # list of target coordinate tuples
        self.target_index = 0
        self.status = "IDLE"              # "IDLE", "FLYING", "INSPECTING", "RETURNING", "LOW_BATTERY"
        self.total_flight_time = 0.0
        self.total_distance_m = 0.0

    def update_physics(self, delta_t=1.0):
        """
        Updates drone position and battery SoC over time step delta_t (seconds).
        """
        if self.battery_soc <= 0.0:
            self.status = "LOW_BATTERY"
            return

        if self.active_route and self.target_index < len(self.active_route):
            target_pos = np.array(self.active_route[self.target_index], dtype=np.float64)
            direction = target_pos - self.pos
            distance = np.linalg.norm(direction)

            if distance < 2.0:  # Reached target radius (2 meters)
                self.target_index += 1
                self.status = "INSPECTING"
            else:
                self.status = "FLYING"
                step_dist = min(distance, self.max_speed * delta_t)
                unit_vec = direction / (distance + 1e-6)
                self.pos += unit_vec * step_dist
                self.total_distance_m += step_dist

            # Battery Depletion Model (Paper B - Hong et al. dynamic payload energy model)
            base_drain_per_sec = 0.08  # 0.08% per second base flight
            payload_drain_per_sec = 0.02 * self.current_payload_kg
            total_drain = (base_drain_per_sec + payload_drain_per_sec) * delta_t
            self.battery_soc = max(0.0, self.battery_soc - total_drain)
            self.total_flight_time += delta_t
        else:
            self.status = "IDLE"


class SimulationEnvironment:
    """
    Discrete-Event Simulation Environment for Multi-UAV Swarms.
    Manages Drones, Target Nodes, Road Network Nodes, and Time Progression.
    """
    def __init__(self, area_size=(1000.0, 1000.0), num_drones=4, num_targets=20):
        self.area_width, self.area_height = area_size
        self.num_drones = num_drones
        self.num_targets = num_targets
        
        # Initialize Drones at Depot (0, 0)
        self.drones = [
            DroneAgent(drone_id=i, start_pos=(0.0, 0.0), max_speed=15.0)
            for i in range(num_drones)
        ]

        # Generate realistic target inspection coordinates
        np.random.seed(42)  # Reproducible setup
        self.targets = [
            (float(np.random.uniform(50, self.area_width - 50)),
             float(np.random.uniform(50, self.area_height - 50)))
            for _ in range(num_targets)
        ]
        self.time_step = 0.0

    def step(self, delta_t=1.0):
        """
        Advances the simulation clock by delta_t seconds.
        """
        self.time_step += delta_t
        for drone in self.drones:
            drone.update_physics(delta_t)

    def is_mission_complete(self):
        """
        Checks if all drones have completed assigned target routes.
        """
        return all(d.status in ["IDLE", "LOW_BATTERY"] and d.target_index >= len(d.active_route) for d in self.drones)
