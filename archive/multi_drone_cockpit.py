import os
import time
import cv2  # pyfly: ignore [missing-import] # type: ignore
import numpy as np  # pyfly: ignore [missing-import] # type: ignore
import pybullet as p  # pyfly: ignore [missing-import] # type: ignore
import pybullet_data  # pyfly: ignore [missing-import] # type: ignore

from modules.perception import YOLOPerception
from modules.decision_engine import AIDecisionEngine

class MultiDroneCockpit:
    """
    Multi-UAV 4-Camera Cockpit & Communication Dashboard Simulator.
    Displays:
    1. Real 3D Physics Swarm Environment in PyBullet.
    2. 2x2 Live Camera Feed Grid showing what all 4 drones see simultaneously.
    3. Real-time YOLOv8 Object Detections & Semantic Mode Labels overlaid on each camera.
    4. Inter-Drone Communication Link Status.
    """
    def __init__(self, num_drones=4):
        self.num_drones = num_drones
        self.perception = YOLOPerception(conf_threshold=0.50)
        self.engine = AIDecisionEngine(conf_threshold=0.50)
        self._init_pybullet()

    def _init_pybullet(self):
        try:
            p.connect(p.GUI)
        except Exception:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        plane_id = p.loadURDF("plane.urdf")

        # Load Aerial City Map Texture onto Ground Plane
        texture_path = os.path.join(os.path.dirname(__file__), "data", "aerial_city_map.jpg")
        if os.path.exists(texture_path):
            texture_id = p.loadTexture(texture_path)
            p.changeVisualShape(plane_id, -1, textureUniqueId=texture_id)
            print("[Cockpit] Applied High-Resolution Aerial City Map Texture to 3D Ground Floor!")

        p.resetDebugVisualizerCamera(cameraDistance=7.0, cameraYaw=45, cameraPitch=-35, cameraTargetPosition=[0, 0, 1.0])

        # Spawn 4 Drones
        self.drone_start_positions = [
            [-2.0, -2.0, 1.5],
            [2.0, -2.0, 1.5],
            [-2.0, 2.0, 1.5],
            [2.0, 2.0, 1.5]
        ]
        self.drone_ids = []
        for pos in self.drone_start_positions:
            col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1])
            vis_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1], rgbaColor=[0.1, 0.5, 0.9, 1.0])
            d_id = p.createMultiBody(baseMass=1.5, baseCollisionShapeIndex=col_shape, baseVisualShapeIndex=vis_shape, basePosition=pos)
            self.drone_ids.append(d_id)

        # Spawn Target Objects (Red Box, Green Box, Blue Box, Yellow Box)
        target_colors = [[0.9, 0.1, 0.1, 1], [0.1, 0.9, 0.1, 1], [0.1, 0.1, 0.9, 1], [0.9, 0.9, 0.1, 1]]
        target_positions = [[-1.5, -1.0, 0.3], [1.5, -1.0, 0.3], [-1.0, 1.5, 0.3], [1.0, 1.5, 0.3]]
        
        for pos, color in zip(target_positions, target_colors):
            t_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.25])
            t_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.25], rgbaColor=color)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=t_col, baseVisualShapeIndex=t_vis, basePosition=pos)

    def render_drone_camera(self, drone_id, step):
        """
        Renders synthetic onboard 3D camera feed for specific drone and overlays YOLOv8 detection.
        """
        drone_pos, _ = p.getBasePositionAndOrientation(drone_id)
        
        # Camera mounted looking downward/forward from drone position
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[drone_pos[0], drone_pos[1], 0.0],
            distance=2.5,
            yaw=45 + (drone_id * 90),
            pitch=-40,
            roll=0,
            upAxisIndex=2
        )
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100.0)
        
        # Capture 240x240 image
        img_cam = p.getCameraImage(240, 240, view_matrix, proj_matrix)
        rgb_array = np.reshape(img_cam[2], (240, 240, 4))[:, :, :3].astype(np.uint8)
        bgr_frame = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)

        # Run YOLOv8 / Perception Evaluation
        boxes, confs, Sk = self.perception.detect_objects(bgr_frame)
        
        # Simulate telemetry for drone
        battery_soc = max(15.0, 100.0 - step * 0.1 - (drone_id * 10))
        bw_bk = 6.0
        drift_var = 18.2 if (step % 40 < 15 and drone_id == 2) else 3.5

        telemetry = {
            "confidence_Sk": Sk,
            "drift_variance": drift_var,
            "battery_soc": battery_soc,
            "bandwidth_bk": bw_bk,
            "drone_id": drone_id + 1
        }
        actions = self.engine.evaluate_state_vector(telemetry)

        # Draw overlays on Camera View
        overlay = bgr_frame.copy()
        
        # Draw detected object bounding boxes
        for box in boxes:
            bx, by, bw_b, bh_b = [int(v) for v in box]
            cv2.rectangle(overlay, (bx - bw_b//2, by - bh_b//2), (bx + bw_b//2, by + bh_b//2), (0, 255, 0), 2)

        # Header overlay
        cv2.rectangle(overlay, (0, 0), (240, 45), (20, 20, 20), -1)
        cv2.putText(overlay, f"DRONE {drone_id+1} CAMERA FEED", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        
        # Mode & Battery status text
        mode_text = "MODE 1: SEMANTIC COMPRESS" if "MODE_1" in actions['comm_mode'] else "MODE 2: RAW OFFLOAD"
        mode_color = (0, 255, 0) if "MODE_1" in actions['comm_mode'] else (0, 165, 255)
        
        cv2.putText(overlay, mode_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, mode_color, 1)
        cv2.putText(overlay, f"Sk:{Sk:.2f} | SoC:{battery_soc:.0f}%", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # Alert overlay if peer drift correction triggered
        if actions['peer_correction_triggered']:
            cv2.putText(overlay, "DRIFT DETECTED -> PEER H(A->B)", (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        return overlay, actions

    def start_cockpit_simulation(self):
        print("===================================================================")
        print("  STARTING MULTI-DRONE 4-CAMERA COCKPIT SIMULATOR")
        print("===================================================================")
        print("[Cockpit] Opening 2x2 Quad-Camera Window displaying all 4 Drone Camera Feeds...")
        print("[Cockpit] Press 'q' or Esc in the OpenCV window to exit.")

        step = 0
        try:
            while p.isConnected():
                # Update PyBullet 3D Physics
                for i, d_id in enumerate(self.drone_ids):
                    hover_thrust = 9.81 * 1.5 + np.sin(step * 0.05 + i) * 1.5
                    p.applyExternalForce(d_id, -1, [0, 0, hover_thrust], [0, 0, 0], p.WORLD_FRAME)
                    vx = np.cos(step * 0.03 + i * 1.5) * 2.0
                    vy = np.sin(step * 0.03 + i * 1.5) * 2.0
                    p.applyExternalForce(d_id, -1, [vx, vy, 0], [0, 0, 0], p.WORLD_FRAME)

                p.stepSimulation()

                # Render 4 Drone Camera Feeds every 5 steps
                if step % 5 == 0:
                    cam_views = []
                    all_actions = []
                    for i, d_id in enumerate(self.drone_ids):
                        cam_frame, act = self.render_drone_camera(i, step)
                        cam_views.append(cam_frame)
                        all_actions.append(act)

                    # Arrange 4 Drone Cameras into 2x2 Grid Window
                    top_row = np.hstack((cam_views[0], cam_views[1]))
                    bot_row = np.hstack((cam_views[2], cam_views[3]))
                    grid_2x2 = np.vstack((top_row, bot_row))

                    # Display Live 4-Camera Cockpit Grid Window
                    cv2.imshow("Multi-UAV Swarm 4-Camera View Cockpit (Live AI Engine)", grid_2x2)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:
                        break

                time.sleep(0.01)
                step += 1

        except KeyboardInterrupt:
            print("\n[Cockpit] Simulation stopped.")
        finally:
            cv2.destroyAllWindows()
            if p.isConnected():
                p.disconnect()
            print("\n[SUCCESS] Cockpit Simulator Closed Cleanly.")

if __name__ == "__main__":
    cockpit = MultiDroneCockpit()
    cockpit.start_cockpit_simulation()
