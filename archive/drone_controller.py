import os
import math
import random
import time
import cv2  # pyfly: ignore [missing-import] # type: ignore
import keyboard  # pyfly: ignore [missing-import] # type: ignore
import numpy as np  # pyfly: ignore [missing-import] # type: ignore
import pybullet as p  # pyfly: ignore [missing-import] # type: ignore
import pybullet_data  # pyfly: ignore [missing-import] # type: ignore

# ─────────────────────────────────────────────────────────────
#  MULTI-UAV CITY MISSION CONTROLLER
# ─────────────────────────────────────────────────────────────

DRONE_COLORS  = [[0.2,0.5,1.0,1],[0.1,0.85,0.1,1],[1.0,0.55,0.1,1],[0.8,0.1,0.9,1]]
DRONE_NAMES   = ["DRONE 1","DRONE 2","DRONE 3","DRONE 4"]
CV_COLORS_BGR = [(230,130,30),(40,210,40),(30,140,255),(220,40,220)]
DEPOT_POS     = [[-4,-4,2],[4,-4,2],[-4,4,2],[4,4,2]]

MASS         = 1.5
GRAVITY      = 9.81
HOVER        = MASS * GRAVITY
MOVE_FORCE   = 22.0
VERT_FORCE   = 28.0
DT           = 1.0 / 60.0

# Internal PyBullet render resolution (low for performance)
RENDER_W     = 240
RENDER_H     = 180
# OpenCV display upscale resolution for a single panel in grid mode
CAM_W        = 640
CAM_H        = 480
RENDER_EVERY = 10

# Waypoints
WAYPOINTS = [
    ( 2.0,  2.0, 3.8), (-2.0,  3.5, 2.5), ( 3.5, -2.0, 4.5), (-3.5, -3.0, 3.0),
    ( 0.5,  5.5, 5.2), (-5.0,  1.0, 2.8), ( 5.0,  3.5, 3.5), (-1.0, -5.0, 4.0),
]

# Humans
HUMAN_PATROL = [
    [( 1.0, 0.0),( 1.0, 2.0),(-1.0, 2.0),(-1.0, 0.0)],
    [(-1.0, 0.0),(-1.0,-2.0),( 1.0,-2.0),( 1.0, 0.0)],
    [( 3.0, 1.0),( 3.0,-1.0),( 5.0,-1.0),( 5.0, 1.0)],
    [(-3.0,-1.0),(-3.0, 1.0),(-5.0, 1.0),(-5.0,-1.0)],
    [( 2.0, 4.0),(-2.0, 4.0),(-2.0, 6.0),( 2.0, 6.0)],
    [(-4.0, 4.0),(-4.0, 6.0),(-6.0, 6.0),(-6.0, 4.0)],
    [( 4.0,-4.0),( 4.0,-6.0),( 6.0,-6.0),( 6.0,-4.0)],
    [(-1.0,-4.0),(-1.0,-6.0),( 1.0,-6.0),( 1.0,-4.0)],
]


class MissionDroneCity:
    def __init__(self):
        self.selected      = 0
        self.view_mode     = "single"  # "single" or "grid"
        self.drone_ids     = []
        self.step          = 0

        self.waypoint_ids  = []
        self.waypoint_pos  = WAYPOINTS[:]
        self.active_wp     = [-1] * 4
        self.auto_fly      = [False] * 4
        self.landed        = [False] * 4

        # Persistent cargo boxes: {'id': body_id, 'carried_by': None | drone_idx}
        self.cargo_boxes   = []

        self.human_ids     = []
        self.human_patrol  = HUMAN_PATROL[:]
        self.human_step    = [0] * len(HUMAN_PATROL)

        self._f_was = self._p_was = self._o_was = self._r_was = self._v_was = False

        self._init_pybullet()
        self._register_hotkeys()

    def _init_pybullet(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -GRAVITY)
        p.setTimeStep(DT)
        p.setRealTimeSimulation(0)

        self._build_city()

        p.resetDebugVisualizerCamera(
            cameraDistance=22, cameraYaw=30,
            cameraPitch=-45, cameraTargetPosition=[0,0,0])

        # Spawn Drones
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

        # Spawn Humans
        for i, patrol in enumerate(HUMAN_PATROL):
            hx, hy = patrol[0]
            body_col = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.18, height=1.0)
            body_vis = p.createVisualShape(p.GEOM_CAPSULE,    radius=0.18, length=1.0,
                                           rgbaColor=[0.8,0.55,0.35,1])
            hid = p.createMultiBody(baseMass=0,
                                    baseCollisionShapeIndex=body_col,
                                    baseVisualShapeIndex=body_vis,
                                    basePosition=[hx, hy, 0.75])
            self.human_ids.append(hid)

    def _build_city(self):
        random.seed(42); np.random.seed(42)
        plane = p.loadURDF("plane.urdf")
        p.changeVisualShape(plane, -1, rgbaColor=[0.18,0.18,0.18,1])

        # Roads
        for axis in range(2):
            he = [9.0,0.18,0.01] if axis==0 else [0.18,9.0,0.01]
            rc = p.createCollisionShape(p.GEOM_BOX, halfExtents=he)
            rv = p.createVisualShape(p.GEOM_BOX,   halfExtents=he, rgbaColor=[0.9,0.9,0.2,1])
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rc,
                              baseVisualShapeIndex=rv, basePosition=[0,0,0.01])

        palette = [
            [0.55,0.60,0.70,1],[0.45,0.50,0.62,1],[0.70,0.65,0.55,1],
            [0.40,0.55,0.70,1],[0.65,0.55,0.45,1],[0.50,0.60,0.65,1],
        ]

        placed = 0; attempts = 0
        while placed < 30 and attempts < 800:
            attempts += 1
            bw = random.uniform(0.8,1.8)
            bd = random.uniform(0.8,1.8)
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

        # Helipads & Initial Cargo Boxes
        for wx, wy, wz in self.waypoint_pos:
            hc = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.55, height=0.08)
            hv = p.createVisualShape(p.GEOM_CYLINDER,   radius=0.55, length=0.08,
                                     rgbaColor=[0.9,0.8,0.1,1.0])
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=hc,
                              baseVisualShapeIndex=hv, basePosition=[wx, wy, wz])
            
            # Spawn a persistent cargo box on every helipad
            bc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15,0.15,0.15])
            bv = p.createVisualShape(p.GEOM_BOX,   halfExtents=[0.15,0.15,0.15], rgbaColor=[1.0,0.5,0.0,1.0])
            cargo_id = p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=bc,
                                         baseVisualShapeIndex=bv, basePosition=[wx, wy, wz+0.25])
            self.cargo_boxes.append({'id': cargo_id, 'carried_by': None})

        # Depots
        for dp in DEPOT_POS:
            hc = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.7, height=0.04)
            hv = p.createVisualShape(p.GEOM_CYLINDER,   radius=0.7, length=0.04,
                                     rgbaColor=[0.85,0.1,0.1,0.9])
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=hc,
                              baseVisualShapeIndex=hv, basePosition=[dp[0],dp[1],0.02])

    def _register_hotkeys(self):
        keyboard.on_press_key('1', lambda _: self._select(0))
        keyboard.on_press_key('2', lambda _: self._select(1))
        keyboard.on_press_key('3', lambda _: self._select(2))
        keyboard.on_press_key('4', lambda _: self._select(3))

    def _select(self, idx):
        self.selected = idx

    def _nearest_waypoint(self, drone_idx):
        try:
            pos, _ = p.getBasePositionAndOrientation(self.drone_ids[drone_idx])
            dists = [math.sqrt((pos[0]-wx)**2 + (pos[1]-wy)**2) for wx,wy,wz in self.waypoint_pos]
            return int(np.argmin(dists))
        except: return 0

    def _autopilot_step(self, drone_idx):
        wp_idx = self.active_wp[drone_idx]
        if wp_idx < 0: return
        wx, wy, wz = self.waypoint_pos[wp_idx]
        target_z = wz + 0.5

        try:
            pos, _ = p.getBasePositionAndOrientation(self.drone_ids[drone_idx])
            dx = wx - pos[0];  dy = wy - pos[1];  dz = target_z - pos[2]
            dist_xy = math.sqrt(dx**2 + dy**2)
            dist_z  = abs(dz)

            kp_xy = 4.0;  kp_z = 6.0
            fx = np.clip(dx * kp_xy, -MOVE_FORCE, MOVE_FORCE)
            fy = np.clip(dy * kp_xy, -MOVE_FORCE, MOVE_FORCE)
            fz_ctrl = np.clip(dz * kp_z, -VERT_FORCE, VERT_FORCE)

            p.applyExternalForce(self.drone_ids[drone_idx], -1,
                                 [fx, fy, HOVER + fz_ctrl], [0,0,0], p.WORLD_FRAME)

            if dist_xy < 0.35 and dist_z < 0.4:
                self.landed[drone_idx] = True
                self.auto_fly[drone_idx] = False
                p.resetBasePositionAndOrientation(
                    self.drone_ids[drone_idx], [wx, wy, wz + 0.12], [0,0,0,1])
                p.resetBaseVelocity(self.drone_ids[drone_idx],[0,0,0],[0,0,0])
        except: pass

    def _pickup(self, drone_idx):
        # Check if already carrying something
        for box in self.cargo_boxes:
            if box['carried_by'] == drone_idx:
                return

        # Find nearest available cargo box
        try:
            pos, _ = p.getBasePositionAndOrientation(self.drone_ids[drone_idx])
            min_dist = 1.0
            target_box = None
            for box in self.cargo_boxes:
                if box['carried_by'] is not None: continue
                bpos, _ = p.getBasePositionAndOrientation(box['id'])
                dist = math.sqrt((pos[0]-bpos[0])**2 + (pos[1]-bpos[1])**2 + (pos[2]-bpos[2])**2)
                if dist < min_dist:
                    min_dist = dist
                    target_box = box
            
            if target_box:
                target_box['carried_by'] = drone_idx
                self.landed[drone_idx] = False
        except: pass

    def _deliver(self, drone_idx):
        for box in self.cargo_boxes:
            if box['carried_by'] == drone_idx:
                box['carried_by'] = None
                try:
                    pos, _ = p.getBasePositionAndOrientation(self.drone_ids[drone_idx])
                    p.resetBasePositionAndOrientation(box['id'], [pos[0],pos[1],pos[2]-0.4], [0,0,0,1])
                except: pass
                return

    def _update_cargo(self):
        for box in self.cargo_boxes:
            drone_idx = box['carried_by']
            if drone_idx is not None:
                try:
                    pos, orn = p.getBasePositionAndOrientation(self.drone_ids[drone_idx])
                    p.resetBasePositionAndOrientation(box['id'], [pos[0],pos[1],pos[2]-0.35], orn)
                    p.resetBaseVelocity(box['id'], [0,0,0], [0,0,0])
                except: pass

    def _update_humans(self):
        speed = 0.012
        for i, hid in enumerate(self.human_ids):
            patrol = self.human_patrol[i]
            ci = self.human_step[i]
            ni = (ci + 1) % len(patrol)
            tx, ty = patrol[ni]
            try:
                cur_pos, _ = p.getBasePositionAndOrientation(hid)
                cx, cy = cur_pos[0], cur_pos[1]
                dx, dy = tx-cx, ty-cy
                dist = math.sqrt(dx**2+dy**2)
                if dist < 0.15:
                    self.human_step[i] = ni
                else:
                    nx = cx + dx/dist*speed
                    ny = cy + dy/dist*speed
                    yaw = math.atan2(dy, dx)
                    orn = p.getQuaternionFromEuler([0, 0, yaw])
                    p.resetBasePositionAndOrientation(hid,[nx,ny,0.75],orn)
            except: pass

    # ── TRUE FPV Camera render ─────────────────────────────────────────
    def _render_camera(self, idx, out_w, out_h):
        try:
            pos, orn = p.getBasePositionAndOrientation(self.drone_ids[idx])
            yaw_deg = 45 + idx * 90
            yaw_rad = math.radians(yaw_deg)
            fx = math.cos(yaw_rad)
            fy = math.sin(yaw_rad)
            
            cam_pos = [pos[0] + fx*0.1, pos[1] + fy*0.1, pos[2] - 0.05]
            target_pos = [cam_pos[0] + fx, cam_pos[1] + fy, cam_pos[2] - 0.3]

            vm = p.computeViewMatrix(
                cameraEyePosition=cam_pos,
                cameraTargetPosition=target_pos,
                cameraUpVector=[0, 0, 1]
            )
            pm = p.computeProjectionMatrixFOV(
                fov=80, aspect=RENDER_W/RENDER_H, nearVal=0.01, farVal=120.0)
            
            raw = p.getCameraImage(RENDER_W, RENDER_H, vm, pm, renderer=p.ER_TINY_RENDERER)
            frame = np.reshape(raw[2],(RENDER_H,RENDER_W,4))[:,:,:3].astype(np.uint8)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return cv2.resize(frame_bgr, (out_w, out_h), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            return np.zeros((out_h, out_w, 3), np.uint8)

    def _draw_hud(self, frame, idx):
        is_sel = (idx == self.selected)
        h, w = frame.shape[:2]
        
        hud = frame.copy()
        cv2.rectangle(hud,(0,0),(w, 75),(20,20,20),-1)
        cv2.addWeighted(hud, 0.6, frame, 0.4, 0, frame)
        
        cv2.putText(frame, DRONE_NAMES[idx],(10,25),
                    cv2.FONT_HERSHEY_SIMPLEX,0.8,CV_COLORS_BGR[idx],2)

        has_cargo = any(box['carried_by'] == idx for box in self.cargo_boxes)

        if self.auto_fly[idx]:
            lbl = f"AUTO-PILOT → WP {self.active_wp[idx]}"
            lbl_col = (0,200,255)
        elif self.landed[idx]:
            lbl = "LANDED [P:Pickup Cargo | O:Drop Cargo]"
            lbl_col = (0,255,120)
        elif has_cargo:
            lbl = "CARGO SECURED [F:AutoFly | O:Drop]"
            lbl_col = (0,200,255)
        elif is_sel:
            lbl = "MANUAL CONTROL [W/A/S/D Q/E:Alt]"
            lbl_col = (0,255,255)
        else:
            lbl = "HOVERING"
            lbl_col = (140,140,140)

        cv2.putText(frame, lbl,(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.55,lbl_col,1)

        try:
            pos,_ = p.getBasePositionAndOrientation(self.drone_ids[idx])
            vel,_ = p.getBaseVelocity(self.drone_ids[idx])
            spd   = float(np.linalg.norm(vel[:3]))
            cargo_str = "[CARGO]" if has_cargo else ""
            cv2.putText(frame,f"ALT: {pos[2]:.1f}m  SPD: {spd:.1f}m/s  {cargo_str}",
                        (10,70),cv2.FONT_HERSHEY_SIMPLEX,0.45,(200,200,200),1)
        except: pass

        # FPV Crosshair
        cx,cy = w//2, h//2
        col = (0,255,255) if is_sel else (70,70,70)
        cv2.line(frame,(cx-20,cy),(cx-5,cy),col,2)
        cv2.line(frame,(cx+5,cy),(cx+20,cy),col,2)
        cv2.line(frame,(cx,cy-20),(cx,cy-5),col,2)
        cv2.line(frame,(cx,cy+5),(cx,cy+20),col,2)
        cv2.circle(frame,(cx,cy),2,col,-1)
        
        cv2.line(frame,(cx-40,cy-30),(cx-20,cy-30),col,1)
        cv2.line(frame,(cx+20,cy-30),(cx+40,cy-30),col,1)
        cv2.line(frame,(cx-40,cy+30),(cx-20,cy+30),col,1)
        cv2.line(frame,(cx+20,cy+30),(cx+40,cy+30),col,1)

        if is_sel:
            cv2.rectangle(frame,(0,0),(w-1,h-1),(0,255,255),4)
        return frame

    def _draw_minimap(self, frame):
        h, w = frame.shape[:2]
        map_size = 220
        margin = 20
        x0, y0 = w - map_size - margin, margin
        
        # Draw background
        cv2.rectangle(frame, (x0, y0), (x0+map_size, y0+map_size), (30,30,30), -1)
        cv2.rectangle(frame, (x0, y0), (x0+map_size, y0+map_size), (150,150,150), 2)
        cv2.putText(frame, "CITY RADAR", (x0+5, y0+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
        
        # Coordinate mapper: PyBullet (-10 to 10) to map (0 to map_size)
        def world_to_map(wx, wy):
            mx = int(x0 + (wx + 10) / 20.0 * map_size)
            my = int(y0 + (10 - wy) / 20.0 * map_size)
            return mx, my

        # Waypoints (Helipads)
        for wx, wy, _ in self.waypoint_pos:
            mx, my = world_to_map(wx, wy)
            cv2.circle(frame, (mx, my), 6, (0, 150, 150), -1)

        # Cargo Boxes
        for box in self.cargo_boxes:
            if box['carried_by'] is None:
                try:
                    bpos, _ = p.getBasePositionAndOrientation(box['id'])
                    mx, my = world_to_map(bpos[0], bpos[1])
                    cv2.rectangle(frame, (mx-4, my-4), (mx+4, my+4), (0, 140, 255), -1)
                except: pass

        # Drones
        for i, did in enumerate(self.drone_ids):
            try:
                pos, _ = p.getBasePositionAndOrientation(did)
                mx, my = world_to_map(pos[0], pos[1])
                col = CV_COLORS_BGR[i]
                radius = 7 if i == self.selected else 4
                cv2.circle(frame, (mx, my), radius, col, -1)
                
                yaw_deg = 45 + i * 90
                yaw_rad = math.radians(yaw_deg)
                fx = math.cos(yaw_rad) * 12
                fy = math.sin(yaw_rad) * 12
                cv2.line(frame, (mx, my), (int(mx + fx), int(my - fy)), col, 2)
            except: pass

        return frame

    def _build_ui(self):
        full_w = CAM_W * 2
        full_h = CAM_H * 2
        
        if self.view_mode == "grid":
            views = [self._draw_hud(self._render_camera(i, CAM_W, CAM_H), i) for i in range(4)]
            top  = np.hstack((views[0],views[1]))
            bot  = np.hstack((views[2],views[3]))
            grid = np.vstack((top,bot))
        else:
            # Single view
            view = self._render_camera(self.selected, full_w, full_h)
            view = self._draw_hud(view, self.selected)
            view = self._draw_minimap(view)
            grid = view
            
        bar = np.zeros((36, full_w, 3), np.uint8)
        cv2.putText(bar,
            f"V: Toggle View (Grid/Single)  |  1-4: Select Drone  |  WASD+QE: Fly  |  F: Auto-Land  |  P: Pickup Cargo  |  O: Drop Cargo  |  ESC: Quit",
            (10,23), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,180), 1)
        return np.vstack((grid, bar))

    def run(self):
        WIN = "Multi-UAV True FPV Cockpit"
        
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        grid = self._build_ui()

        while True:
            if not p.isConnected(): break

            if keyboard.is_pressed('esc'): break

            # Toggle View Mode
            v_now = keyboard.is_pressed('v')
            if v_now and not self._v_was:
                self.view_mode = "grid" if self.view_mode == "single" else "single"
            self._v_was = v_now

            r_now = keyboard.is_pressed('r')
            if r_now and not self._r_was:
                did = self.drone_ids[self.selected]
                p.resetBasePositionAndOrientation(did, DEPOT_POS[self.selected],[0,0,0,1])
                p.resetBaseVelocity(did,[0,0,0],[0,0,0])
                self.auto_fly[self.selected] = False
                self.landed[self.selected]   = False
            self._r_was = r_now

            f_now = keyboard.is_pressed('f')
            if f_now and not self._f_was:
                wp = self._nearest_waypoint(self.selected)
                self.active_wp[self.selected]  = wp
                self.auto_fly[self.selected]   = True
                self.landed[self.selected]     = False
            self._f_was = f_now

            p_now = keyboard.is_pressed('p')
            if p_now and not self._p_was:
                self._pickup(self.selected)
            self._p_was = p_now

            o_now = keyboard.is_pressed('o')
            if o_now and not self._o_was:
                self._deliver(self.selected)
            self._o_was = o_now

            for i, did in enumerate(self.drone_ids):
                if self.auto_fly[i]:
                    self._autopilot_step(i)
                else:
                    p.applyExternalForce(did,-1,[0,0,HOVER],[0,0,0],p.WORLD_FRAME)
                    if i == self.selected:
                        f_fwd = f_right = fz = 0.0
                        if keyboard.is_pressed('w'): f_fwd += MOVE_FORCE
                        if keyboard.is_pressed('s'): f_fwd -= MOVE_FORCE
                        if keyboard.is_pressed('a'): f_right -= MOVE_FORCE
                        if keyboard.is_pressed('d'): f_right += MOVE_FORCE
                        if keyboard.is_pressed('q'): fz += VERT_FORCE
                        if keyboard.is_pressed('e'): fz -= VERT_FORCE
                        
                        if f_fwd != 0.0 or f_right != 0.0 or fz != 0.0:
                            self.landed[i] = False
                            yaw_rad = math.radians(45 + i * 90)
                            fx = f_fwd * math.cos(yaw_rad) + f_right * math.sin(yaw_rad)
                            fy = f_fwd * math.sin(yaw_rad) - f_right * math.cos(yaw_rad)
                            p.applyExternalForce(did,-1,[fx,fy,fz],[0,0,0],p.WORLD_FRAME)

            self._update_cargo()

            if self.step % 3 == 0:
                self._update_humans()

            p.stepSimulation()

            if self.step % RENDER_EVERY == 0:
                grid = self._build_ui()
                cv2.imshow(WIN, grid)
                cv2.waitKey(1)

            self.step += 1

        keyboard.unhook_all()
        cv2.destroyAllWindows()
        try: p.disconnect()
        except: pass

if __name__ == "__main__":
    MissionDroneCity().run()
