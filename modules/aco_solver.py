import math
import random
import copy

class ACOSolver:
    """
    Ant Colony Optimization (ACO) for Multi-Drone Task Allocation and Routing.
    Allocates packages to drones minimizing total distance/makespan.
    """
    def __init__(self, num_ants=20, num_iterations=50, alpha=1.0, beta=2.0, evaporation_rate=0.1):
        self.num_ants = num_ants
        self.num_iterations = num_iterations
        self.alpha = alpha
        self.beta = beta
        self.evaporation_rate = evaporation_rate
        
    def distance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

    def solve(self, drones, packages):
        """
        drones: dict {drone_id: (x, y, z)}
        packages: dict {pkg_id: {'start': (x, y, z), 'dest': (x, y, z)}}
        Returns: {drone_id: [pkg_id_1, pkg_id_2, ...]}
        """
        if not packages:
            return {d_id: [] for d_id in drones}
            
        pkg_ids = list(packages.keys())
        drone_ids = list(drones.keys())
        
        # Pheromone matrix mapping (from_node, to_pkg)
        # Nodes are either drone_ids (start) or pkg_ids (after delivery)
        nodes = drone_ids + pkg_ids
        pheromones = {n: {p: 1.0 for p in pkg_ids} for n in nodes}
        
        best_solution = None
        best_cost = float('inf')
        
        for iteration in range(self.num_iterations):
            solutions = []
            costs = []
            
            for ant in range(self.num_ants):
                # Build a solution
                unvisited = set(pkg_ids)
                ant_solution = {d_id: [] for d_id in drone_ids}
                # Track current position of each drone
                drone_pos = {d_id: drones[d_id] for d_id in drone_ids}
                # Track the last node the drone visited (to lookup pheromone)
                drone_last_node = {d_id: d_id for d_id in drone_ids}
                # Track total distance per drone (makespan)
                drone_dist = {d_id: 0.0 for d_id in drone_ids}
                
                while unvisited:
                    # Pick the drone with the minimum distance traveled so far (load balancing)
                    current_drone = min(drone_ids, key=lambda d: drone_dist[d])
                    
                    last_n = drone_last_node[current_drone]
                    pos = drone_pos[current_drone]
                    
                    # Calculate probabilities to pick next package
                    probabilities = {}
                    prob_sum = 0.0
                    
                    for pkg in unvisited:
                        pkg_start = packages[pkg]['start']
                        pkg_dest = packages[pkg]['dest']
                        
                        # Distance to pickup + distance to dropoff
                        dist_to_serve = self.distance(pos, pkg_start) + self.distance(pkg_start, pkg_dest)
                        
                        # Heuristic: inverse of distance
                        eta = 1.0 / (dist_to_serve + 0.1)
                        tau = pheromones[last_n][pkg]
                        
                        prob = (tau ** self.alpha) * (eta ** self.beta)
                        probabilities[pkg] = prob
                        prob_sum += prob
                        
                    # Select next package based on probability
                    rand = random.uniform(0, prob_sum)
                    cumulative = 0.0
                    selected_pkg = None
                    
                    for pkg, prob in probabilities.items():
                        cumulative += prob
                        if cumulative >= rand:
                            selected_pkg = pkg
                            break
                    if selected_pkg is None: # Fallback
                        selected_pkg = list(unvisited)[0]
                        
                    # Update ant state
                    unvisited.remove(selected_pkg)
                    ant_solution[current_drone].append(selected_pkg)
                    
                    # Distance traveled
                    dist = self.distance(pos, packages[selected_pkg]['start']) + self.distance(packages[selected_pkg]['start'], packages[selected_pkg]['dest'])
                    drone_dist[current_drone] += dist
                    drone_pos[current_drone] = packages[selected_pkg]['dest']
                    drone_last_node[current_drone] = selected_pkg
                
                # Evaluate solution: max distance among drones (Makespan)
                max_cost = max(drone_dist.values())
                solutions.append(ant_solution)
                costs.append(max_cost)
                
                if max_cost < best_cost:
                    best_cost = max_cost
                    best_solution = copy.deepcopy(ant_solution)
                    
            # Evaporate pheromones
            for n in nodes:
                for p in pkg_ids:
                    pheromones[n][p] *= (1.0 - self.evaporation_rate)
                    
            # Deposit pheromones (elitist - only best solution deposits)
            if best_solution and best_cost > 0:
                deposit_amount = 100.0 / best_cost
                for d_id, route in best_solution.items():
                    last_node = d_id
                    for pkg in route:
                        pheromones[last_node][pkg] += deposit_amount
                        last_node = pkg
                    
        # Compute total swarm distance
        total_swarm_dist = 0.0
        if best_solution:
            for d_id, route in best_solution.items():
                curr_pos = drones[d_id]
                for pkg in route:
                    p_start = packages[pkg]['start']
                    p_dest = packages[pkg]['dest']
                    total_swarm_dist += self.distance(curr_pos, p_start) + self.distance(p_start, p_dest)
                    curr_pos = p_dest

        return best_solution, best_cost, total_swarm_dist

    def export_route_report(self, best_solution, drones, packages, filepath="logs/aco_optimal_routes.txt"):
        """Exports a human-readable and verifiable ACO optimal route report."""
        import os
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        with open(filepath, "w", encoding="utf-8") as f:
            f.write("=" * 70 + "\n")
            f.write("  ANT COLONY OPTIMIZATION (ACO) - MULTI-UAV OPTIMAL ROUTE REPORT\n")
            f.write("  Autonomous Swarm Delivery System | Capstone Review Verification\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"Algorithm Parameters:\n")
            f.write(f"  • Number of Ants       : {self.num_ants}\n")
            f.write(f"  • ACO Iterations       : {self.num_iterations}\n")
            f.write(f"  • Pheromone Factor (α) : {self.alpha}\n")
            f.write(f"  • Heuristic Factor (β) : {self.beta}\n")
            f.write(f"  • Evaporation Rate (ρ) : {self.evaporation_rate}\n\n")
            f.write("-" * 70 + "\n")
            f.write("OPTIMAL TASK ASSIGNMENTS & TRAJECTORY PATHS:\n")
            f.write("-" * 70 + "\n")

            total_dist = 0.0
            drone_distances = {}

            for d_id, route in best_solution.items():
                f.write(f"\n[{d_id.upper()}] Assigned Packages: {len(route)} packages\n")
                curr_pos = drones[d_id]
                d_dist = 0.0
                f.write(f"  Start Depot / Pad: ({curr_pos[0]:.1f}, {curr_pos[1]:.1f}, {curr_pos[2]:.1f})\n")
                
                for step, pkg in enumerate(route, start=1):
                    p_start = packages[pkg]['start']
                    p_dest = packages[pkg]['dest']
                    leg1 = self.distance(curr_pos, p_start)
                    leg2 = self.distance(p_start, p_dest)
                    d_dist += leg1 + leg2
                    f.write(f"    Step {step}: Pickup {pkg} at ({p_start[0]:.1f}, {p_start[1]:.1f}, {p_start[2]:.1f}) "
                            f"--> Deliver to Dest ({p_dest[0]:.1f}, {p_dest[1]:.1f}, {p_dest[2]:.1f}) "
                            f"[Leg: {leg1 + leg2:.1f}m]\n")
                    curr_pos = p_dest
                    
                drone_distances[d_id] = d_dist
                total_dist += d_dist
                f.write(f"  Total Flight Distance for {d_id}: {d_dist:.2f} meters\n")

            f.write("\n" + "=" * 70 + "\n")
            f.write("GLOBAL OPTIMIZATION METRICS:\n")
            f.write(f"  • Total Swarm Flight Distance : {total_dist:.2f} meters\n")
            makespan = max(drone_distances.values()) if drone_distances else 0.0
            f.write(f"  • Optimal Makespan (Max Drone): {makespan:.2f} meters\n")
            f.write(f"  • Average Load Per Drone      : {total_dist / max(1, len(drones)):.2f} meters\n")
            f.write("=" * 70 + "\n")
