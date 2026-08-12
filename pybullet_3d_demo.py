import time
import cv2  # pyfly: ignore [missing-import] # type: ignore
import numpy as np  # pyfly: ignore [missing-import] # type: ignore
import pybullet as p  # pyfly: ignore [missing-import] # type: ignore
import pybullet_data  # pyfly: ignore [missing-import] # type: ignore
from modules.perception import YOLOPerception

def run_pybullet_3d_drone_sim():
    """
    Launches a real 3D PyBullet Physics Environment with Drone Camera Object Detection (YOLOv8).
    """
    print("===================================================================")
    print("  LAUNCHING 3D PYBULLET MULTI-UAV PHYSICS & VISION SIMULATOR")
    print("===================================================================")

    try:
        physicsClient = p.connect(p.GUI)
        print("[PyBullet] Connected to 3D OpenGL GUI Window.")
    except Exception as e:
        physicsClient = p.connect(p.DIRECT)
        print(f"[PyBullet] Connected in Headless DIRECT Mode: {e}")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load 3D Ground Plane
    planeId = p.loadURDF("plane.urdf")
    print("[PyBullet] 3D Terrain Plane Loaded.")

    # Initialize YOLOv8 Edge Perception Model
    perception = YOLOPerception(conf_threshold=0.50)

    # Set camera position to focus clearly on the drone swarm
    p.resetDebugVisualizerCamera(cameraDistance=6.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 1.5])

    # Spawn 4 3D Drones
    drone_start_positions = [
        [-1.5, -1.5, 1.0],
        [1.5, -1.5, 1.2],
        [-1.5, 1.5, 1.4],
        [1.5, 1.5, 1.1]
    ]

    drone_ids = []
    for i, pos in enumerate(drone_start_positions):
        col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1])
        vis_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1], rgbaColor=[0.1, 0.5, 0.9, 1.0])
        drone_id = p.createMultiBody(baseMass=1.5, baseCollisionShapeIndex=col_shape, baseVisualShapeIndex=vis_shape, basePosition=pos)
        drone_ids.append(drone_id)

    # Add 3D visual target objects (Red target blocks representing inspection objects)
    target_positions = [[-2.5, 1.8, 0.3], [2.5, -1.8, 0.3], [1.8, 2.5, 0.3], [-1.8, -2.5, 0.3]]
    for t_pos in target_positions:
        t_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3])
        t_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.3], rgbaColor=[0.9, 0.1, 0.1, 1.0])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=t_col, baseVisualShapeIndex=t_vis, basePosition=t_pos)

    print(f"[PyBullet] Spawned {len(drone_ids)} Drones & 4 Target Objects.")
    print("[PyBullet] Capturing Live Onboard Drone Camera Feed & Running YOLOv8 Detection...")

    step = 0
    try:
        while p.isConnected():
            for i, d_id in enumerate(drone_ids):
                hover_thrust = 9.81 * 1.5 + np.sin(step * 0.05 + i) * 1.5
                p.applyExternalForce(d_id, -1, [0, 0, hover_thrust], [0, 0, 0], p.WORLD_FRAME)
                
                vx = np.cos(step * 0.03 + i * 1.5) * 2.2
                vy = np.sin(step * 0.03 + i * 1.5) * 2.2
                p.applyExternalForce(d_id, -1, [vx, vy, 0], [0, 0, 0], p.WORLD_FRAME)

            # Render Synthetic Camera Feed from Drone 1's perspective
            if step % 20 == 0:
                drone_pos, drone_orn = p.getBasePositionAndOrientation(drone_ids[0])
                view_matrix = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=drone_pos,
                    distance=3.0,
                    yaw=45,
                    pitch=-30,
                    roll=0,
                    upAxisIndex=2
                )
                proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100.0)
                
                # Capture camera image array
                img_camera = p.getCameraImage(320, 320, view_matrix, proj_matrix)
                rgb_array = np.reshape(img_camera[2], (320, 320, 4))[:, :, :3].astype(np.uint8)
                rgb_bgr = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)

                # Run live YOLOv8 object detection on drone camera feed
                boxes, confs, Sk = perception.detect_objects(rgb_bgr)
                comm_info = perception.select_compression_mode(Sk, raw_image=rgb_bgr)

                print(f"[Drone 1 Camera @ Step {step}] Detected Objects: {len(boxes)} | Sk: {Sk:.2f} | Mode: {comm_info['mode']}")

            p.stepSimulation()
            time.sleep(0.01)
            step += 1

    except KeyboardInterrupt:
        print("\n[PyBullet] Simulation stopped by user.")
    finally:
        if p.isConnected():
            p.disconnect()
        print("\n===================================================================")
        print("  [SUCCESS] 3D PyBullet Multi-UAV Physics & Vision Simulation Complete!")
        print("===================================================================")

if __name__ == "__main__":
    run_pybullet_3d_drone_sim()
