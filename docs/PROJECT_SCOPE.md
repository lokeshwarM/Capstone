# Capstone Project: Comprehensive Project Scope

**Project Title:** Adaptive Multi-UAV Swarm Delivery & Inspection with Ground Mothership Coordination, Semantic Communication, and Dual-Layer Route Optimization (ACO + ALNS)  
**Institution:** School of Computer Science and Engineering (SCOPE), VIT-AP University  
**Academic Year:** 2025–2026  

---

## 1. Executive Summary & Problem Statement

Modern last-mile delivery and smart-city infrastructure inspection increasingly face severe urban congestion, strict delivery deadlines, and energy constraints. While Unmanned Aerial Vehicles (UAVs / Drones) offer superior agility, standalone drones are severely limited by battery capacity and payload weight. Furthermore, multi-UAV swarms operating in dense urban environments suffer from:
1. **Communication Bandwidth Bottlenecks:** Streaming high-resolution visual feeds (4K/1080p) from multiple drones quickly saturates wireless channels.
2. **Visual Odometry & Sensor Drift:** High-speed flights between urban high-rises cause optical flow tracking loss and GPS signal degradation.
3. **Dynamic Operational Disturbances:** Premature battery exhaustion or sudden task cancellation requires instantaneous re-routing without mission failure.

To address these challenges, this project develops an integrated, autonomous **Heterogeneous Multi-UAV Swarm System** coordinated with a mobile **Ground Mothership Delivery Truck**, combining computer vision, semantic communication, bio-inspired optimization, and event-driven AI decision-making.

---

## 2. In-Scope Deliverables

The scope of this project encompasses the design, mathematical formulation, software implementation, and simulation validation of the following subsystems:

### 2.1. Mobile Ground Mothership & Swarm Coordination
* **Mobile Mothership Truck:** Acts as a mobile depot, battery recharging base, and localized cargo distribution hub that moves dynamically through city corridors.
* **4-Drone Landing Deck:** Dedicated physical landing pads on the truck roof enabling simultaneous landing, fast inductive recharging, and independent takeoff without starvation.
* **Multi-Tier Parcel Stacking System:** 15 distinct packages organized into 5 multi-tiered stacks in the truck cargo bed, picked in an automated top-down sequence by the drone swarm.

### 2.2. Edge Vision Perception & Adaptive Semantic Communication
* **YOLOv8 Aerial Object Detection:** Real-time downward-facing camera perception for detecting ground targets, obstacles, and delivery landing helipads.
* **Lucas-Kanade (LK) Optical Flow Tracking:** Continuous drift variance monitoring to detect camera tilt, motion blur, or aerodynamic turbulence.
* **Semantic Compression Switcher:** Dynamically selects communication payloads based on detection confidence ($S_k$) and channel bandwidth ($b_k$):
  * **Full Image Mode:** Transmitted only when high-confidence new targets are detected ($S_k \ge 0.70$).
  * **Feature/Crop Mode:** Transmits only segmented regions of interest (ROI) when bandwidth drops ($S_k \in [0.45, 0.70)$), achieving $>75\%$ data reduction.
  * **Metadata/Bounding Box Mode:** Transmits raw coordinates and telemetry ($<2\text{ KB}$) under critical channel degradation.

### 2.3. Event-Driven AI Decision Engine
* **State Vector Evaluation:** Real-time multi-parameter evaluation combining:
  $$\mathbf{S}(t) = \big[S_k(t), \ \sigma^2_{\text{drift}}(t), \ \text{SoC}_{\text{batt}}(t), \ b_k(t)\big]$$
* **Peer Drift Correction Protocol:** When a drone detects abnormal optical flow drift ($\sigma^2 > 15.0$), it broadcasts a D2D correction beacon to neighboring swarm drones to triangulate and correct heading.
* **Low-Battery Emergency Protection:** Triggers graceful descent or emergency return when state of charge drops below safety margins ($<20\%$).

### 2.4. Dual-Layer Route Optimization (ACO + ALNS)
* **Macro-Layer: Ant Colony Optimization (ACO):**
  * Solves the multi-depot, multi-vehicle routing problem (VRP) before mission execution.
  * Formulates artificial pheromone evaporation ($\rho$) and heuristic distance visibility ($\eta$) to assign package sequences across the swarm, minimizing total makespan and energy consumption.
* **Micro-Layer: Adaptive Large Neighborhood Search (ALNS):**
  * Real-time dynamic ruin-and-repair heuristic triggered during in-flight battery degradation or unexpected mission halts.
  * Removes orphaned delivery packages from failing drones and greedily inserts them into surviving neighbor drone routes without halting the global fleet.

### 2.5. High-Fidelity 3D Physics Simulation (ROS 2 & Gazebo Classic)
* Complete 3D urban environment (`city.sdf`) with multi-story buildings, rooftop helipads, textured asphalt streets, zebra crosswalks, concrete sidewalks, trees, streetlights, detailed vehicles, and pedestrians.
* ROS 2 Humble integration with asynchronous services (`/set_entity_state`, `/spawn_entity`, `/delete_entity`) and inter-process messaging topics.

---

## 3. Assumptions & Delimitations (Out-of-Scope)

To maintain scientific rigor and adhere to project timeline and laboratory safety regulations, the following boundaries are defined:

1. **Outdoor Physical Airspace Flights (Regulatory Constraints):**
   * *Delimitation:* Physical multi-drone flights in outdoor public airspace are excluded due to regulatory compliance and licensing restrictions under the Directorate General of Civil Aviation (DGCA) and local drone airspace rules.
   * *Mitigation:* Validated using ROS 2 with physics-accurate Gazebo 11 dynamics, simulated wind/drift parameters, and real camera image pipelines.
2. **Extreme Atmospheric Weather Disturbances:**
   * *Delimitation:* Modeling severe meteorological events (e.g., cyclonic downbursts, torrential monsoon rain, icing) is excluded.
   * *Assumption:* Environmental wind and turbulence are modeled as stochastic Gaussian drift noise in the optical flow tracker.
3. **Dynamic Airspace Non-Cooperative Obstacles:**
   * *Delimitation:* Dynamic bird strikes or malicious rogue drones entering the delivery corridor are out of scope. All aerial agents belong to the cooperative swarm.
4. **Hardware Battery Chemistry & Thermal Dissipation:**
   * *Assumption:* Battery discharge follows standard linear-distance energy drain profiles with payload-mass coefficients, omitting electrochemical cell degradation and internal temperature rise.

---

## 4. Future Scope & Research Horizons

The architecture is designed to be modular, paving the way for future research extensions:

1. **5G-NR Sidelink / V2X Wireless Testbed:**
   * Transitioning from ROS 2 DDS middleware to cellular Vehicle-to-Everything (C-V2X) PC5 direct communication for ultra-reliable low-latency drone-to-drone communication (URLLC).
2. **PX4 Hardware-in-the-Loop (HIL) Integration:**
   * Connecting the decision engine and ACO/ALNS planner to PX4 Autopilot hardware running on physical companion computers (Raspberry Pi 5 / NVIDIA Jetson Orin Nano).
3. **Deep Reinforcement Learning (DRL) for Continuous Airspace Control:**
   * Augmenting the discrete ACO planner with continuous Multi-Agent Deep Deterministic Policy Gradient (MADDPG) or PPO for fluid collision avoidance in non-grid cities.
4. **LSTM-Based Neural Dead-Reckoning:**
   * Deploying sequence-to-sequence Long Short-Term Memory (LSTM) models for real-time trajectory forecasting and path continuation during GPS denial and visual occlusion in deep urban canyons.
