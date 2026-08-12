import numpy as np  # pyfly: ignore [missing-import] # type: ignore
import scipy.spatial.distance as sp_dist  # pyfly: ignore [missing-import] # type: ignore

class ACOAllocator:
    """
    Ant Colony Optimization (ACO) Task Allocation Module (Paper 1 - Xu et al.).
    Allocates N inspection targets among K drones under energy & distance constraints.
    """
    def __init__(self, num_drones=4, max_tasks_per_drone=5, alpha=1.0, beta=2.0, rho=0.1):
        self.num_drones = num_drones
        self.max_tasks_per_drone = max_tasks_per_drone
        self.alpha = alpha  # Pheromone importance
        self.beta = beta    # Heuristic distance importance
        self.rho = rho      # Pheromone evaporation rate

    def solve(self, drone_positions, target_positions, iterations=20):
        """
        Runs ACO optimization to find optimal drone-target assignment.
        Returns:
            assignments: dict mapping drone_id -> list of assigned target indices
            total_distance: float total flight distance
        """
        num_targets = len(target_positions)
        if num_targets == 0:
            return {i: [] for i in range(self.num_drones)}, 0.0

        # Calculate distance matrix between drones and targets
        dist_matrix = np.zeros((self.num_drones, num_targets))
        for i in range(self.num_drones):
            for j in range(num_targets):
                dist_matrix[i, j] = np.linalg.norm(
                    np.array(drone_positions[i]) - np.array(target_positions[j])
                )

        # Initialize Pheromone matrix
        pheromone = np.ones((self.num_drones, num_targets))
        best_assignment = None
        best_total_dist = float('inf')

        for _ in range(iterations):
            current_assignment = {i: [] for i in range(self.num_drones)}
            unassigned_targets = list(range(num_targets))

            # Assign targets greedily based on pheromone & heuristic
            for target_idx in unassigned_targets:
                probs = []
                for drone_id in range(self.num_drones):
                    if len(current_assignment[drone_id]) < self.max_tasks_per_drone:
                        eta = 1.0 / (dist_matrix[drone_id, target_idx] + 1e-6)
                        prob = (pheromone[drone_id, target_idx] ** self.alpha) * (eta ** self.beta)
                    else:
                        prob = 0.0
                    probs.append(prob)

                probs = np.array(probs)
                if np.sum(probs) > 0:
                    probs = probs / np.sum(probs)
                    chosen_drone = np.random.choice(self.num_drones, p=probs)
                else:
                    chosen_drone = target_idx % self.num_drones

                current_assignment[chosen_drone].append(target_idx)

            # Compute total flight distance
            current_dist = 0.0
            for drone_id, targets in current_assignment.items():
                curr_pos = np.array(drone_positions[drone_id])
                for t_idx in targets:
                    t_pos = np.array(target_positions[t_idx])
                    current_dist += np.linalg.norm(curr_pos - t_pos)
                    curr_pos = t_pos

            if current_dist < best_total_dist:
                best_total_dist = current_dist
                best_assignment = current_assignment

            # Update Pheromones (Evaporation + Deposit)
            pheromone *= (1.0 - self.rho)
            if best_assignment:
                for drone_id, targets in best_assignment.items():
                    for t_idx in targets:
                        pheromone[drone_id, t_idx] += 1.0 / (best_total_dist + 1e-6)

        return best_assignment, best_total_dist


class ALNSRouter:
    """
    Adaptive Large Neighborhood Search (ALNS) Dynamic Re-Routing (Paper 2 & Paper B).
    Ruin-and-Recreate operator to handle battery depletion or new emergency triggers.
    """
    def __init__(self):
        pass

    def destroy_and_repair(self, current_routes, low_battery_drone_id):
        """
        Removes remaining tasks from low-battery drone and inserts them into active neighbor routes.
        """
        updated_routes = {k: list(v) for k, v in current_routes.items()}
        
        if low_battery_drone_id in updated_routes:
            orphaned_tasks = updated_routes[low_battery_drone_id]
            updated_routes[low_battery_drone_id] = []  # Clear route for failed drone
            
            # Find active candidate drones
            active_drones = [d for d in updated_routes.keys() if d != low_battery_drone_id]
            if active_drones and orphaned_tasks:
                # Greedily distribute orphaned tasks to the active drone with fewest tasks
                for task in orphaned_tasks:
                    target_drone = min(active_drones, key=lambda d: len(updated_routes[d]))
                    updated_routes[target_drone].append(task)
                    
        return updated_routes
