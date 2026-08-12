import os
import random
import time
import cv2  # pyfly: ignore [missing-import] # type: ignore
import keyboard  # pyfly: ignore [missing-import] # type: ignore
import numpy as np  # pyfly: ignore [missing-import] # type: ignore
import pybullet as p  # pyfly: ignore [missing-import] # type: ignore
import pybullet_data  # pyfly: ignore [missing-import] # type: ignore

# ─────────────────────────────────────────────────────────────
#  CONTROLS — works in ANY window, no need to click anything!
#
#  1 / 2 / 3 / 4  → Select Drone to fly
#  HOLD W / S      → Forward / Backward
#  HOLD A / D      → Left / Right
#  HOLD Q / E      → Up / Down
#  TAP  R          → Return selected drone to depot
#  TAP  ESC        → Quit
# ─────────────────────────────────────────────────────────────

DRONE_COLORS  = [[0.2,0.5,1.0,1],[0.1,0.85,0.1,1],[1.0,0.55,0.1,1],[0.8,0.1,0.9,1]]
DRONE_NAMES   = ["DRONE 1","DRONE 2","DRONE 3","DRONE 4"]
CV_COLORS_BGR = [(230,130,30),(40,210,40),(30,140,255),(220,40,220)]
DEPOT_POS     = [[-4,-4,2],[4,-4,2],[-4,4,2],[4,4,2]]

MASS          = 1.5
GRAVITY       = 9.81
HOVER         = MASS * GRAVITY
MOVE_FORCE    = 22.0
VERT_FORCE    = 28.0
DT            = 1.0 / 60.0
CAM_W, CAM_H  = 320, 240
RENDER_EVERY  = 10

def build_3d_city():
    random.seed(42); np.random.seed(42)

    plane_id = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane_id, -1, rgbaColor=[0.18,0.18,0.18,1])

    # Road cross
    for axis in range(2):
        he = [10.0,0.18,0.01] if axis==0 else [0.18,10.0,0.01]
        rc = p.createCollisionShape(p.GEOM_BOX, halfExtents=he)
        rv = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=[0.9,0.9,0.2,1])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rc,
                          baseVisualShapeIndex=rv, basePosition=[0,0,0.01])

    palette = [
        [0.55,0.60,0.70,1],[0.45,0.50,0.62,1],[0.70,0.65,0.55,1],
        [0.40,0.55,0.70,1],[0.65,0.55,0.45,1],[0.50,0.60,0.65,1],
    ]
    placed = 0; attempts = 0
    while placed < 35 and attempts < 800:
        attempts += 1
        bw = random.uniform(0.7,1.8); bd = random.uniform(0.7,1.8)
        bh = random.uniform(1.0,5.5)
        bx = random.uniform(-8+bw, 8-bw)
        by = random.uniform(-8+bd, 8-bd)
        if abs(bx)<2.0 and abs(by)<2.0: continue
        if any(abs(bx-d[0])<3.0 and abs(by-d[1])<3.0 for d in DEPOT_POS): continue
        col = palette[placed%len(palette)]
        bc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[bw/2,bd/2,bh/2])
        bv = p.createVisualShape(p.GEOM_BOX,  halfExtents=[bw/2,bd/2,bh/2], rgbaColor=col)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=bc,
                          baseVisualShapeIndex=bv, basePosition=[bx,by,bh/2])
        placed += 1

    # Helipads + colour markers
    for i, dp in enumerate(DEPOT_POS):
        hc = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.7, height=0.04)
        hv = p.createVisualShape(p.GEOM_CYLINDER,   radius=0.7, length=0.04,
                                  rgbaColor=[0.85,0.1,0.1,0.9])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=hc,
                          baseVisualShapeIndex=hv, basePosition=[dp[0],dp[1],0.02])
        mc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3,0.3,0.05])
        mv = p.createVisualShape(p.GEOM_BOX,  halfExtents=[0.3,0.3,0.05],
                                  rgbaColor=DRONE_COLORS[i])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=mc,
                          baseVisualShapeIndex=mv, basePosition=[dp[0],dp[1],0.06])

    print(f"[City] 3D city built: {placed} buildings, roads, 4 helipads.")


class DroneControlSim:
    def __init__(self):
        self.selected  = 0
        self.drone_ids = []
        self.step      = 0
        self._r_was_pressed = False   # edge detection for R
        self._init_pybullet()
        self._register_hotkeys()
        self._print_controls()

    def _init_pybullet(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -GRAVITY)
        p.setTimeStep(DT)
        p.setRealTimeSimulation(0)

        build_3d_city()
        p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=30,
                                      cameraPitch=-45, cameraTargetPosition=[0,0,0])

        for i, pos in enumerate(DEPOT_POS):
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.28,0.28,0.08])
            vis = p.createVisualShape(p.GEOM_BOX,   halfExtents=[0.28,0.28,0.08],
                                      rgbaColor=DRONE_COLORS[i])
            did = p.createMultiBody(baseMass=MASS,
                                    baseCollisionShapeIndex=col,
                                    baseVisualShapeIndex=vis,
                                    basePosition=pos)
            p.changeDynamics(did, -1, linearDamping=0.92, angularDamping=0.99)
            self.drone_ids.append(did)
        print("[Sim] PyBullet environment ready.")

    def _register_hotkeys(self):
        """Register tap-once hotkeys for drone selection & depot return."""
        keyboard.on_press_key('1', lambda _: self._select(0))
        keyboard.on_press_key('2', lambda _: self._select(1))
        keyboard.on_press_key('3', lambda _: self._select(2))
        keyboard.on_press_key('4', lambda _: self._select(3))

    def _select(self, idx):
        self.selected = idx
        print(f"[>] Switched to {DRONE_NAMES[idx]}")

    def _print_controls(self):
        print("\n" + "="*58)
        print("  MULTI-DRONE MANUAL CONTROL — 3D CITY")
        print("="*58)
        print("  Keys work EVERYWHERE — no window focus needed!")
        print()
        print("  TAP  1/2/3/4  Select drone")
        print("  HOLD W / S    Forward / Backward")
        print("  HOLD A / D    Left / Right")
        print("  HOLD Q / E    Up / Down")
        print("  TAP  R        Return to depot")
        print("  TAP  ESC      Quit")
        print("="*58 + "\n")

    def _render_camera(self, idx):
        try:
            pos, _ = p.getBasePositionAndOrientation(self.drone_ids[idx])
        except Exception:
            return np.zeros((CAM_H, CAM_W, 3), np.uint8)
        vm = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[pos[0], pos[1], pos[2]-1.2],
            distance=3.0, yaw=45+idx*90, pitch=-30, roll=0, upAxisIndex=2)
        pm = p.computeProjectionMatrixFOV(
            fov=70, aspect=CAM_W/CAM_H, nearVal=0.05, farVal=120.0)
        raw = p.getCameraImage(CAM_W, CAM_H, vm, pm, renderer=p.ER_TINY_RENDERER)
        frame = np.reshape(raw[2], (CAM_H, CAM_W, 4))[:,:,:3].astype(np.uint8)
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def _draw_hud(self, frame, idx):
        is_sel = (idx == self.selected)
        cv2.rectangle(frame, (0,0), (CAM_W, 52), (10,10,10), -1)
        cv2.putText(frame, DRONE_NAMES[idx], (7,16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, CV_COLORS_BGR[idx], 1)
        lbl = ">>> HOLD W/A/S/D to fly, Q/E=Alt <<<" if is_sel else "Hovering"
        lbl_col = (0,255,255) if is_sel else (140,140,140)
        cv2.putText(frame, lbl, (7,32), cv2.FONT_HERSHEY_SIMPLEX, 0.33, lbl_col, 1)
        try:
            pos,_ = p.getBasePositionAndOrientation(self.drone_ids[idx])
            vel,_ = p.getBaseVelocity(self.drone_ids[idx])
            spd   = float(np.linalg.norm(vel[:3]))
            cv2.putText(frame, f"Alt:{pos[2]:.1f}m  Spd:{spd:.1f}m/s",
                        (7,48), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (200,200,200), 1)
        except Exception:
            pass
        cx, cy = CAM_W//2, CAM_H//2
        col = (0,255,255) if is_sel else (70,70,70)
        cv2.line(frame,(cx-14,cy),(cx+14,cy),col,1)
        cv2.line(frame,(cx,cy-14),(cx,cy+14),col,1)
        cv2.circle(frame,(cx,cy),5,col,1)
        if is_sel:
            cv2.rectangle(frame,(0,0),(CAM_W-1,CAM_H-1),(0,255,255),3)
        return frame

    def _build_grid(self):
        views = [self._draw_hud(self._render_camera(i), i) for i in range(4)]
        top  = np.hstack((views[0], views[1]))
        bot  = np.hstack((views[2], views[3]))
        grid = np.vstack((top, bot))
        bar  = np.zeros((34, CAM_W*2, 3), np.uint8)
        cv2.putText(bar,
            f"Flying: {DRONE_NAMES[self.selected]}  |  WASD/QE=Move  1-4=Select  R=Depot  ESC=Quit",
            (4,22), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255,255,180), 1)
        return np.vstack((grid, bar))

    def run(self):
        WIN = "4-Drone Camera Feeds — Live"
        grid = self._build_grid()

        while True:
            if not p.isConnected():
                break

            # ── ESC to quit (global) ──────────────────────────────
            if keyboard.is_pressed('esc'):
                print("[Sim] ESC — quitting.")
                break

            # ── Return to depot (R) ───────────────────────────────
            if keyboard.is_pressed('r'):
                if not self._r_was_pressed:
                    did = self.drone_ids[self.selected]
                    p.resetBasePositionAndOrientation(
                        did, DEPOT_POS[self.selected], [0,0,0,1])
                    p.resetBaseVelocity(did, [0,0,0], [0,0,0])
                    print(f"[>] {DRONE_NAMES[self.selected]} → depot")
                    self._r_was_pressed = True
            else:
                self._r_was_pressed = False

            # ── Apply hover to ALL drones ─────────────────────────
            for i, did in enumerate(self.drone_ids):
                p.applyExternalForce(did, -1, [0,0,HOVER], [0,0,0], p.WORLD_FRAME)

                # Directional force only on selected drone
                if i == self.selected:
                    fx = fy = fz = 0.0
                    if keyboard.is_pressed('w'): fx += MOVE_FORCE
                    if keyboard.is_pressed('s'): fx -= MOVE_FORCE
                    if keyboard.is_pressed('a'): fy -= MOVE_FORCE
                    if keyboard.is_pressed('d'): fy += MOVE_FORCE
                    if keyboard.is_pressed('q'): fz += VERT_FORCE
                    if keyboard.is_pressed('e'): fz -= VERT_FORCE
                    if fx or fy or fz:
                        p.applyExternalForce(did, -1, [fx,fy,fz],
                                             [0,0,0], p.WORLD_FRAME)

            # ── Physics step ──────────────────────────────────────
            p.stepSimulation()

            # ── Camera grid display ───────────────────────────────
            if self.step % RENDER_EVERY == 0:
                grid = self._build_grid()
                cv2.imshow(WIN, grid)
                cv2.waitKey(1)

            self.step += 1

        keyboard.unhook_all()
        cv2.destroyAllWindows()
        try:
            p.disconnect()
        except Exception:
            pass
        print("[Sim] Closed cleanly.")


if __name__ == "__main__":
    DroneControlSim().run()
