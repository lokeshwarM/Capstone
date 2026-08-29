# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from gazebo_msgs.srv import SetEntityState, SpawnEntity, DeleteEntity
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import Image
# pyrefly: ignore [missing-import]
from gazebo_msgs.msg import ModelStates
# pyrefly: ignore [missing-import]
from cv_bridge import CvBridge
import sys
import os
import termios
import tty
import select
import threading
import queue
# pyrefly: ignore [missing-import]
import cv2
import math
import time
import random

# ──────────────────────────────────────────────────────────────────────────────
# Help text
# ──────────────────────────────────────────────────────────────────────────────
HELP_MSG = """
╔══════════════════════════════════════════════════════════════╗
║           DRONE DELIVERY SIMULATION — CONTROLS               ║
╠══════════════════════════════════════════════════════════════╣
║  SELECT DRONE    │  Keys 1 / 2 / 3 / 4                      ║
╠══════════════════════════════════════════════════════════════╣
║  DRONE MOVEMENT  │  W=Forward(+X)   S=Backward(-X)          ║
║  (MANUAL mode)   │  A=Left(+Y)      D=Right(-Y)             ║
║                  │  Q=Up(+Z)        E=Down(-Z)               ║
║                  │  T=Takeoff/Land toggle                    ║
╠══════════════════════════════════════════════════════════════╣
║  DRONE ACTIONS   │  P=Pick package from truck                ║
║  (MANUAL mode)   │  X=Drop held package at destination       ║
╠══════════════════════════════════════════════════════════════╣
║  TRUCK CONTROL   │  ↑ Arrow = Forward (+X)                   ║
║  (ALWAYS works)  │  ↓ Arrow = Backward (-X)                  ║
║                  │  ← Arrow = Left (+Y)                      ║
║                  │  → Arrow = Right (-Y)                     ║
╠══════════════════════════════════════════════════════════════╣
║  MODE TOGGLE     │  M = Switch MANUAL ↔ AUTO (ACO)          ║
║  QUIT            │  CTRL-C                                   ║
╚══════════════════════════════════════════════════════════════╝
AUTO mode: Drones communicate, pick by battery priority, deliver, return.
"""

# ──────────────────────────────────────────────────────────────────────────────
# 15 visually distinct package colours (RGB 0-1)
# ──────────────────────────────────────────────────────────────────────────────
PKG_COLORS = [
    (1.00, 0.10, 0.10),  # Vivid Red
    (0.10, 0.80, 0.10),  # Vivid Green
    (0.10, 0.30, 1.00),  # Vivid Blue
    (1.00, 0.90, 0.00),  # Yellow
    (1.00, 0.10, 0.80),  # Magenta
    (0.00, 0.90, 0.90),  # Cyan
    (1.00, 0.50, 0.00),  # Orange
    (0.60, 0.00, 1.00),  # Violet
    (0.00, 0.60, 0.30),  # Dark Teal
    (1.00, 0.20, 0.50),  # Hot Pink
    (0.00, 0.50, 1.00),  # Sky Blue
    (0.60, 1.00, 0.00),  # Lime
    (0.55, 0.27, 0.07),  # Brown
    (0.00, 0.80, 0.60),  # Mint
    (0.90, 0.60, 0.10),  # Gold
]

# ──────────────────────────────────────────────────────────────────────────────
# Drone visual colours matching route lines
# ──────────────────────────────────────────────────────────────────────────────
DRONE_COLORS = {
    'drone1': (1.0, 0.2, 0.2),   # Red
    'drone2': (0.2, 1.0, 0.2),   # Green
    'drone3': (0.3, 0.5, 1.0),   # Blue
    'drone4': (1.0, 0.9, 0.1),   # Yellow
}

# ──────────────────────────────────────────────────────────────────────────────
# SDF helpers
# ──────────────────────────────────────────────────────────────────────────────

def _box_sdf(name, sx, sy, sz, r, g, b, a=1.0, static=True, gravity=False):
    st = "true" if static else "false"
    gv = "0" if not gravity else "1"
    return (f'<?xml version="1.0"?><sdf version="1.6">'
            f'<model name="{name}"><static>{st}</static>'
            f'<link name="link"><gravity>{gv}</gravity>'
            f'<visual name="v"><geometry><box>'
            f'<size>{sx} {sy} {sz}</size></box></geometry>'
            f'<material><ambient>{r} {g} {b} {a}</ambient>'
            f'<diffuse>{r} {g} {b} {a}</diffuse></material></visual>'
            f'<collision name="c"><geometry><box>'
            f'<size>{sx} {sy} {sz}</size></box></geometry></collision>'
            f'</link></model></sdf>')

def _cylinder_sdf(name, radius, length, r, g, b, a=1.0, static=True):
    st = "true" if static else "false"
    return (f'<?xml version="1.0"?><sdf version="1.6">'
            f'<model name="{name}"><static>{st}</static>'
            f'<link name="link">'
            f'<visual name="v"><geometry><cylinder>'
            f'<radius>{radius}</radius><length>{length}</length>'
            f'</cylinder></geometry>'
            f'<material><ambient>{r} {g} {b} {a}</ambient>'
            f'<diffuse>{r} {g} {b} {a}</diffuse></material></visual>'
            f'</link></model></sdf>')

# ──────────────────────────────────────────────────────────────────────────────
# Keyboard input thread — handles both regular keys AND arrow escape sequences
# ──────────────────────────────────────────────────────────────────────────────

def keyboard_reader_thread(orig_settings, key_q: queue.Queue):
    ARROW_MAP = {
        '\x1b[A': 'ARROW_UP',
        '\x1b[B': 'ARROW_DOWN',
        '\x1b[C': 'ARROW_RIGHT',
        '\x1b[D': 'ARROW_LEFT',
    }
    fd = sys.stdin.fileno()
    while True:
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not rlist:
                termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
                continue

            ch = sys.stdin.read(1)

            if ch == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist2:
                    seq = sys.stdin.read(2)
                    termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
                    full = ch + seq
                    mapped = ARROW_MAP.get(full)
                    if mapped:
                        key_q.put(mapped)
                else:
                    termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
            else:
                termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
                key_q.put(ch)
        except Exception:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
            except Exception:
                pass

# ──────────────────────────────────────────────────────────────────────────────
# Main Node
# ──────────────────────────────────────────────────────────────────────────────

class TeleopNode(Node):

    DESTINATIONS = [
        (-45.0, -45.0, 10.8), (-45.0, -15.0, 27.7), (-30.0,  30.0, 27.5),
        (-15.0,   0.0, 10.3), (  0.0, -15.0, 38.4), (  0.0,  15.0, 19.4),
        ( 15.0, -15.0, 30.2), ( 30.0, -45.0, 35.1),
    ]

    TRUCK_START  = [0.0, -37.5, 1.5]
    TRUCK_SPEED  = 3.0
    CRUISE_ALT   = 50.0
    PICKUP_HOVER = 5.0

    def __init__(self):
        super().__init__('teleop_node')

        self.set_client    = self.create_client(SetEntityState, '/set_entity_state')
        self.spawn_client  = self.create_client(SpawnEntity,    '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity,   '/delete_entity')

        self.get_logger().info('Waiting for /set_entity_state …')
        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  still waiting…')
        self.get_logger().info('Services ready.')

        self.truck_pos = list(self.TRUCK_START)

        # Positions are populated dynamically from /model_states
        self.current_positions = {}
        self.drone_name = 'drone1'
        self.holding = None

        self.batteries = {f'drone{i}': 100.0 for i in range(1, 5)}
        self.packages = {}

        # ── ACO / Auto ────────────────────────────────────────────────────────
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        try:
            from modules.aco_solver import ACOSolver
            self.aco_solver = ACOSolver()
        except ImportError as e:
            self.aco_solver = None

        self.mode = 'manual'
        self.last_mode_toggle_time = 0.0

        # Start states as RETURNING_TO_TRUCK so they naturally fly from city center
        self.drone_states   = {f'drone{i}': 'RETURNING_TO_TRUCK' for i in range(1, 5)}
        self.drone_targets  = {f'drone{i}': None   for i in range(1, 5)}
        self.drone_payloads = {f'drone{i}': None   for i in range(1, 5)}

        self._pkg_lock = threading.Lock()
        self.box_poses = {}
        self.model_sub = self.create_subscription(ModelStates, '/model_states', self._model_cb, 10)

        self.bridge = CvBridge()
        self.frames = {f'drone{i}': None for i in range(1, 5)}
        for i in range(1, 5):
            self.create_subscription(Image, f'/drone{i}/downward_camera/image_raw',
                                     lambda msg, d=f'drone{i}': self._image_cb(msg, d), 10)
            cv2.namedWindow(f'Drone {i} View', cv2.WINDOW_NORMAL)
            cv2.resizeWindow(f'Drone {i} View', 320, 240)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _model_cb(self, msg):
        try:
            for i, name in enumerate(msg.name):
                if name.startswith('package_') or name == 'delivery_truck':
                    self.box_poses[name] = msg.pose[i]
                elif name in self.drone_states:
                    if name not in self.current_positions:
                        # Initialize from reality (no teleporting!)
                        p = msg.pose[i].position
                        self.current_positions[name] = [p.x, p.y, p.z]
                        self.get_logger().info(f'Initialized {name} at ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})')
        except Exception:
            pass

    def _image_cb(self, msg, drone_id):
        try:
            self.frames[drone_id] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    # ── Camera HUD ────────────────────────────────────────────────────────────

    def show_cameras(self):
        key = -1
        for d_id, frame in self.frames.items():
            if frame is None:
                continue
            frame = frame.copy()
            h, w = frame.shape[:2]

            cr, cg, cb = DRONE_COLORS[d_id]
            border_color = (int(cb*255), int(cg*255), int(cr*255))

            if d_id == self.drone_name:
                cv2.rectangle(frame, (0, 0), (w-1, h-1), (0, 255, 0), 4)
            else:
                cv2.rectangle(frame, (0, 0), (w-1, h-1), border_color, 2)

            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx-10, cy), (cx+10, cy), (0, 200, 255), 1)
            cv2.line(frame, (cx, cy-10), (cx, cy+10), (0, 200, 255), 1)

            pos   = self.current_positions.get(d_id, [0, 0, 0])
            batt  = self.batteries[d_id]
            state = self.drone_states[d_id]
            payload = self.drone_payloads.get(d_id) or (self.holding if d_id == self.drone_name else None)
            pkg_str   = f' ▲{payload}' if payload else ''
            mode_str  = 'AUTO' if self.mode == 'auto' else 'MAN'
            
            # Use shorter abbreviations for HUD
            abbr = state.split('_')[0][:4]
            if state == 'WAITING_FOR_TRUCK': abbr = 'WAIT'
            if state == 'CLIMBING_FROM_TRUCK': abbr = 'CLMB'

            hud = (f'[{mode_str}][{abbr}]'
                   f' ({pos[0]:.0f},{pos[1]:.0f},{pos[2]:.0f})'
                   f' {batt:.0f}%{pkg_str}')

            bg, fg = ((0, 0, 200), (255, 255, 255)) if batt < 20.0 else ((20, 20, 20), (180, 255, 180))
            cv2.rectangle(frame, (0, h-24), (w, h), bg, -1)
            cv2.putText(frame, hud, (4, h-7), cv2.FONT_HERSHEY_SIMPLEX, 0.36, fg, 1)

            if payload and payload in self.packages:
                pr, pg, pb = self.packages[payload]['color']
                swatch = (int(pb*255), int(pg*255), int(pr*255))
                cv2.rectangle(frame, (w-20, h-24), (w, h), swatch, -1)

            cv2.imshow(f'Drone {d_id[-1]} View', frame)
            k = cv2.waitKey(1)
            if k != -1: key = k
        return key

    # ── Pose helpers ──────────────────────────────────────────────────────────

    def _spawn(self, name, xml, x, y, z, qz=0.0, qw=1.0):
        del_req = DeleteEntity.Request()
        del_req.name = name
        self.delete_client.call_async(del_req)
        time.sleep(0.06)

        req = SpawnEntity.Request()
        req.name = name
        req.xml  = xml
        req.initial_pose.position.x    = float(x)
        req.initial_pose.position.y    = float(y)
        req.initial_pose.position.z    = float(z)
        req.initial_pose.orientation.z = float(qz)
        req.initial_pose.orientation.w = float(qw)
        req.reference_frame = 'world'
        self.spawn_client.call_async(req)

    def _set_entity_pose(self, name, x, y, z):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x    = float(x)
        req.state.pose.position.y    = float(y)
        req.state.pose.position.z    = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame    = 'world'
        self.set_client.call_async(req)

    def _set_pose(self, x, y, z):
        if self.drone_name in self.current_positions:
            self._set_entity_pose(self.drone_name, x, y, z)
            if self.holding:
                data = self.packages.get(self.holding, {})
                if not data.get('delivered'):
                    self._set_entity_pose(self.holding, x, y, z - 0.6)

    def _set_pose_auto(self, d_id, x, y, z):
        self._set_entity_pose(d_id, x, y, z)
        payload = self.drone_payloads.get(d_id)
        if payload:
            data = self.packages.get(payload, {})
            if not data.get('delivered') and not data.get('on_truck'):
                self._set_entity_pose(payload, x, y, z - 0.6)

    def _drain(self, d_id, dist, has_payload=False):
        rate = 0.015 + (0.025 if has_payload else 0.0)
        self.batteries[d_id] = max(0.0, self.batteries[d_id] - dist * rate)

    # ── Manual drone movement ─────────────────────────────────────────────────

    def move(self, dx, dy, dz):
        if self.drone_name not in self.current_positions:
            return
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        self._drain(self.drone_name, dist, bool(self.holding))
        pos = self.current_positions[self.drone_name]
        pos[0] += dx
        pos[1] += dy
        pos[2] += dz
        self._set_pose(pos[0], pos[1], pos[2])

    def toggle_takeoff(self):
        if self.drone_name not in self.current_positions:
            return
        z = self.current_positions[self.drone_name][2]
        target = 50.0 if z < 5.0 else 0.5
        self.move(0, 0, target - z)

    def switch_drone(self, n):
        self.drone_name = f'drone{n}'
        if self.drone_name in self.current_positions:
            pos = self.current_positions[self.drone_name]
            self._set_pose(pos[0], pos[1], pos[2])
            self.get_logger().info(f'Selected {self.drone_name}')

    def update_held_object(self):
        if self.holding and self.drone_name in self.current_positions:
            data = self.packages.get(self.holding, {})
            if not data.get('delivered'):
                pos = self.current_positions[self.drone_name]
                self._set_entity_pose(self.holding, pos[0], pos[1], pos[2] - 0.6)

    # ── Truck control ─────────────────────────────────────────────────────────

    def _move_truck(self, dx, dy):
        self.truck_pos[0] += dx
        self.truck_pos[1] += dy
        tx, ty, tz = self.truck_pos
        self._set_entity_pose('delivery_truck', tx, ty, tz)

        for pkg, data in self.packages.items():
            if data.get('on_truck') and not data.get('delivered'):
                sx, sy, sz = self._pkg_slot_pos(data['slot'])
                self._set_entity_pose(pkg, sx, sy, sz)

    def _pkg_slot_pos(self, slot):
        tx, ty, tz = self.truck_pos
        col = slot % 3
        row = slot // 3
        sx = tx - 2.5 + col * 1.2
        sy = ty - 1.2 + row * 0.6
        sz = tz + 1.8  # elevated above truck bed to avoid physics collision
        return sx, sy, sz

    # ── Mission setup ─────────────────────────────────────────────────────────

    def setup_mission(self):
        self.get_logger().info('Setting up mission (YOLO drones will be retained)...')

        # ── Spawn delivery truck ──────────────────────────────────────────────
        truck_xml = (
            '<?xml version="1.0"?><sdf version="1.6">'
            '<model name="delivery_truck"><static>false</static>'
            '<link name="link"><gravity>0</gravity>'
            '<visual name="body"><geometry><box><size>8.0 3.5 2.5</size></box></geometry>'
            '<material><ambient>1.0 0.45 0.0 1</ambient><diffuse>1.0 0.45 0.0 1</diffuse></material></visual>'
            '<collision name="c_body"><geometry><box><size>8.0 3.5 2.5</size></box></geometry></collision>'
            '<visual name="cab"><pose>3.8 0 1.4 0 0 0</pose><geometry><box><size>2.2 3.2 2.0</size></box></geometry>'
            '<material><ambient>0.85 0.35 0.0 1</ambient><diffuse>0.85 0.35 0.0 1</diffuse></material></visual>'
            '<visual name="ws"><pose>4.7 0 2.1 0.35 0 0</pose><geometry><box><size>0.1 2.8 1.3</size></box></geometry>'
            '<material><ambient>0.5 0.8 1.0 0.6</ambient><diffuse>0.5 0.8 1.0 0.6</diffuse></material></visual>'
            '</link></model></sdf>'
        )
        self._spawn('delivery_truck', truck_xml, self.truck_pos[0], self.truck_pos[1], self.truck_pos[2])
        time.sleep(0.4)

        # Note: We DO NOT spawn the visual drone models anymore to preserve the YOLOv8 drones.

        # ── Spawn 15 packages on truck ────────────────────────────────────────
        self.packages = {}
        random.seed(None)

        for i in range(1, 16):
            pkg_name = f'package_{i}'
            dest  = random.choice(self.DESTINATIONS)
            slot  = i - 1
            color = PKG_COLORS[(i - 1) % len(PKG_COLORS)]
            r, g, b = color

            sx, sy, sz = self._pkg_slot_pos(slot)

            self.packages[pkg_name] = {
                'dest':       dest,
                'color':      color,
                'slot':       slot,
                'on_truck':   True,
                'claimed_by': None,
                'delivered':  False,
            }

            # static=True ensures gazebo physics gravity/collisions do not pop them off the truck
            pkg_xml = _box_sdf(pkg_name, 0.85, 0.85, 0.85,
                               r, g, b, 1.0, static=True, gravity=False)
            self._spawn(pkg_name, pkg_xml, sx, sy, sz)
            time.sleep(0.04)

        self.get_logger().info('Mission ready: truck + 15 packages spawned.')

    # ── Manual pick / drop ────────────────────────────────────────────────────

    def pick_object(self):
        if self.holding or self.drone_name not in self.current_positions:
            return

        pos = self.current_positions[self.drone_name]
        tx, ty = self.truck_pos[0], self.truck_pos[1]
        if math.sqrt((pos[0]-tx)**2 + (pos[1]-ty)**2) > 7.0:
            self.get_logger().info('Too far from truck. Fly closer.')
            return

        with self._pkg_lock:
            for pkg_name, data in self.packages.items():
                if not data['delivered'] and data['on_truck'] and data['claimed_by'] is None:
                    data['claimed_by'] = self.drone_name
                    data['on_truck']   = False
                    self.holding       = pkg_name
                    break

        if not self.holding:
            return

        dest = self.packages[self.holding]['dest']
        cr, cg, cb = self.packages[self.holding]['color']
        
        marker_xml = _cylinder_sdf(f'marker_{self.holding}', 1.5, 20.0, cr, cg, cb, 0.6)
        self._spawn(f'marker_{self.holding}', marker_xml, dest[0], dest[1], dest[2] + 10.0)

    def drop_object(self):
        if not self.holding or self.drone_name not in self.current_positions:
            return

        pkg = self.holding
        self.holding = None

        del_req = DeleteEntity.Request()
        del_req.name = f'marker_{pkg}'
        self.delete_client.call_async(del_req)

        dest = self.packages[pkg]['dest']
        pos  = self.current_positions[self.drone_name]
        
        if math.sqrt((pos[0]-dest[0])**2 + (pos[1]-dest[1])**2) < 5.0:
            self.packages[pkg]['delivered']  = True
            self.packages[pkg]['claimed_by'] = None
        else:
            with self._pkg_lock:
                self.packages[pkg]['on_truck']   = True
                self.packages[pkg]['claimed_by'] = None
            sx, sy, sz = self._pkg_slot_pos(self.packages[pkg]['slot'])
            self._set_entity_pose(pkg, sx, sy, sz)

    # ── Auto mode & Queuing ───────────────────────────────────────────────────

    def toggle_mode(self):
        now = time.time()
        if now - self.last_mode_toggle_time < 0.5:
            return
        self.last_mode_toggle_time = now

        if self.mode == 'manual':
            self.mode = 'auto'
            self.get_logger().info('★ AUTO MODE activated')
            for d_id in self.drone_states:
                # Ensure they fly from their CURRENT real positions
                if self.drone_states[d_id] == 'IDLE' and not self.drone_payloads.get(d_id):
                    self.drone_states[d_id] = 'RETURNING_TO_TRUCK'
        else:
            self.mode = 'manual'
            self.get_logger().info('★ MANUAL MODE activated')
            for d_id in self.drone_states:
                self._del_route(d_id)

    def _next_unclaimed(self):
        for pkg, data in self.packages.items():
            if data['on_truck'] and not data['delivered'] and data['claimed_by'] is None:
                return pkg
        return None

    def get_truck_occupant(self):
        """Returns the drone_id currently descending, picking, or climbing from truck."""
        occupying = ['APPROACHING_TRUCK', 'DESCENDING_TO_TRUCK', 'IDLE', 'PICKING_UP', 'CLIMBING_FROM_TRUCK']
        for d_id, state in self.drone_states.items():
            if state in occupying:
                return d_id
        return None

    def _draw_route(self, d_id, sx, sy, ex, ey):
        dx, dy = ex - sx, ey - sy
        dist   = math.sqrt(dx*dx + dy*dy)
        if dist < 0.5: return
        mx, my = sx + dx/2, sy + dy/2
        yaw    = math.atan2(dy, dx)
        cr, cg, cb = DRONE_COLORS[d_id]
        xml = _box_sdf(f'route_{d_id}', dist, 0.6, 0.15, cr, cg, cb, 0.8, static=True)
        self._spawn(f'route_{d_id}', xml, mx, my, self.CRUISE_ALT, math.sin(yaw/2), math.cos(yaw/2))

    def _del_route(self, d_id):
        req = DeleteEntity.Request()
        req.name = f'route_{d_id}'
        self.delete_client.call_async(req)

    def _fly_to_target(self, d_id, tx, ty, tz, horiz_tol, next_state, has_payload):
        pos = self.current_positions[d_id]
        dx, dy = tx - pos[0], ty - pos[1]
        horiz = math.sqrt(dx*dx + dy*dy)
        step = 0.0

        if horiz > horiz_tol:
            # Climb or Cruise
            if pos[2] < self.CRUISE_ALT - 0.5:
                step = min(1.5, self.CRUISE_ALT - pos[2])
                pos[2] += step
            else:
                step = min(1.5, horiz)
                pos[0] += step * dx / horiz
                pos[1] += step * dy / horiz
        else:
            # Descend
            z_diff = tz - pos[2]
            if abs(z_diff) > 0.5:
                step = min(1.2, abs(z_diff))
                pos[2] += math.copysign(step, z_diff)
            else:
                if next_state not in ['PICKING_UP', 'WAITING_FOR_TRUCK', 'RESTING']:
                    self._del_route(d_id)
                self.drone_states[d_id] = next_state

        self._drain(d_id, step, has_payload)
        self._set_pose_auto(d_id, pos[0], pos[1], pos[2])

    def auto_tick(self):
        """Intelligent queuing state machine based on AI engine specs."""
        
        # ── 1. Queue Management (Battery Priority Yielding) ──
        occupant = self.get_truck_occupant()
        if occupant is None:
            waiting = [d for d, st in self.drone_states.items() if st == 'WAITING_FOR_TRUCK']
            if waiting:
                # Lowest battery gets access first! Others yield.
                waiting.sort(key=lambda d: self.batteries[d])
                winner = waiting[0]
                self.drone_states[winner] = 'APPROACHING_TRUCK'
                self.get_logger().info(f'[COORDINATION] {winner} won truck access (lowest batt: {self.batteries[winner]:.1f}%).')

        # ── 2. State Advancements ──
        for d_id in list(self.drone_states.keys()):
            if d_id not in self.current_positions or self.batteries[d_id] <= 0:
                continue

            state = self.drone_states[d_id]
            pos   = self.current_positions[d_id]
            has_payload = bool(self.drone_payloads.get(d_id))

            if state == 'RETURNING_TO_TRUCK':
                tx, ty = self.truck_pos[0], self.truck_pos[1]
                dx, dy = tx - pos[0], ty - pos[1]
                if math.sqrt(dx*dx + dy*dy) < 15.0:
                    self.drone_states[d_id] = 'WAITING_FOR_TRUCK'
                else:
                    self._fly_to_target(d_id, tx, ty, self.CRUISE_ALT, 15.0, 'WAITING_FOR_TRUCK', has_payload)

            elif state == 'WAITING_FOR_TRUCK':
                # Hover and wait in the sky for truck access
                self._drain(d_id, 0.0, has_payload)

            elif state == 'APPROACHING_TRUCK':
                # Line up directly above truck
                tx, ty = self.truck_pos[0], self.truck_pos[1]
                self._fly_to_target(d_id, tx, ty, self.CRUISE_ALT, 0.5, 'DESCENDING_TO_TRUCK', has_payload)

            elif state == 'DESCENDING_TO_TRUCK':
                tx, ty, tz = self.truck_pos
                self._fly_to_target(d_id, tx, ty, tz + self.PICKUP_HOVER, 0.5, 'IDLE', has_payload)

            elif state == 'IDLE':
                pkg = self._next_unclaimed()
                if pkg:
                    with self._pkg_lock:
                        self.packages[pkg]['claimed_by'] = d_id
                    self.drone_payloads[d_id] = pkg
                    self.drone_targets[d_id] = self.packages[pkg]['dest']
                    self.drone_states[d_id] = 'PICKING_UP'
                else:
                    self.drone_states[d_id] = 'RESTING'

            elif state == 'RESTING':
                # No packages left! Find personal resting slot on truck.
                rest_offsets = {'drone1': (2,1), 'drone2': (2,-1), 'drone3': (-2,1), 'drone4': (-2,-1)}
                rx, ry = rest_offsets.get(d_id, (0,0))
                tx, ty, tz = self.truck_pos
                self._fly_to_target(d_id, tx + rx, ty + ry, tz + 0.6, 0.5, 'RESTING', False)
                self.batteries[d_id] = min(100.0, self.batteries[d_id] + 0.03) # Slow charge on truck

            elif state == 'PICKING_UP':
                pkg = self.drone_payloads[d_id]
                self.packages[pkg]['on_truck'] = False
                self._set_entity_pose(pkg, pos[0], pos[1], pos[2] - 0.6)

                dest = self.drone_targets[d_id]
                self._draw_route(d_id, pos[0], pos[1], dest[0], dest[1])
                cr, cg, cb = self.packages[pkg]['color']
                marker_xml = _cylinder_sdf(f'marker_{pkg}', 1.5, 20.0, cr, cg, cb, 0.7)
                self._spawn(f'marker_{pkg}', marker_xml, dest[0], dest[1], dest[2] + 10.0)

                self.get_logger().info(f'{d_id}: picked {pkg} → {dest}')
                self.drone_states[d_id] = 'CLIMBING_FROM_TRUCK'

            elif state == 'CLIMBING_FROM_TRUCK':
                # Climb away from truck to clear the bottleneck for others
                self._fly_to_target(d_id, pos[0], pos[1], self.CRUISE_ALT, 0.5, 'FLYING_TO_DROP', has_payload)

            elif state == 'FLYING_TO_DROP':
                dest = self.drone_targets[d_id]
                self._fly_to_target(d_id, dest[0], dest[1], self.CRUISE_ALT, 1.0, 'DESCENDING_TO_DROP', has_payload)

            elif state == 'DESCENDING_TO_DROP':
                dest = self.drone_targets[d_id]
                self._fly_to_target(d_id, dest[0], dest[1], dest[2] + 2.5, 1.0, 'DROPPING', has_payload)

            elif state == 'DROPPING':
                pkg = self.drone_payloads[d_id]
                if pkg:
                    self.packages[pkg]['delivered']  = True
                    self.packages[pkg]['claimed_by'] = None
                    self.drone_payloads[d_id]        = None
                    del_req = DeleteEntity.Request()
                    del_req.name = f'marker_{pkg}'
                    self.delete_client.call_async(del_req)
                    self.get_logger().info(f'{d_id}: ✔ delivered {pkg}')
                self.drone_states[d_id] = 'RETURNING_TO_TRUCK'


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    orig_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    teleop = TeleopNode()

    print(HELP_MSG)

    ros_thread = threading.Thread(target=rclpy.spin, args=(teleop,), daemon=True)
    ros_thread.start()

    key_q = queue.Queue()
    kb_thread = threading.Thread(target=keyboard_reader_thread, args=(orig_settings, key_q), daemon=True)
    kb_thread.start()

    try:
        teleop.setup_mission()

        while True:
            time.sleep(0.015)
            teleop.update_held_object()

            if teleop.mode == 'auto':
                teleop.auto_tick()

            cv_key = teleop.show_cameras()
            k = chr(cv_key).lower() if cv_key != -1 else None

            try:
                term_k = key_q.get_nowait()
            except queue.Empty:
                term_k = None

            if term_k == 'ARROW_UP':
                teleop._move_truck(TeleopNode.TRUCK_SPEED, 0)
                continue
            elif term_k == 'ARROW_DOWN':
                teleop._move_truck(-TeleopNode.TRUCK_SPEED, 0)
                continue
            elif term_k == 'ARROW_LEFT':
                teleop._move_truck(0, TeleopNode.TRUCK_SPEED)
                continue
            elif term_k == 'ARROW_RIGHT':
                teleop._move_truck(0, -TeleopNode.TRUCK_SPEED)
                continue

            if term_k:
                k = term_k.lower() if len(term_k) == 1 else None

            if not k:
                continue

            if k == '\x03': break
            if k == 'm':
                teleop.toggle_mode()
                continue
            if k in ('1', '2', '3', '4'):
                teleop.switch_drone(int(k))
                continue

            if teleop.mode == 'auto':
                continue

            step = 1.5
            if   k == 'w': teleop.move( step, 0, 0)
            elif k == 's': teleop.move(-step, 0, 0)
            elif k == 'a': teleop.move(0,  step, 0)
            elif k == 'd': teleop.move(0, -step, 0)
            elif k == 'q': teleop.move(0, 0,  step)
            elif k == 'e': teleop.move(0, 0, -step)
            elif k == 't': teleop.toggle_takeoff()
            elif k == 'p': teleop.pick_object()
            elif k == 'x': teleop.drop_object()

    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        teleop.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
