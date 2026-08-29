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
            deposit_amount = 100.0 / best_cost
            for d_id, route in best_solution.items():
                last_node = d_id
                for pkg in route:
                    pheromones[last_node][pkg] += deposit_amount
                    last_node = pkg
                    
        return best_solution
