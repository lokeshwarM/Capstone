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
    """
    Runs in a background thread. Reads raw keypresses from stdin and pushes
    them onto key_q as strings:
      - Regular printable char / ctrl-char → pushed as-is  (e.g. 'w', '\x03')
      - Arrow key escape sequence          → 'ARROW_UP', 'ARROW_DOWN',
                                             'ARROW_LEFT', 'ARROW_RIGHT'
    """
    ARROW_MAP = {
        '\x1b[A': 'ARROW_UP',
        '\x1b[B': 'ARROW_DOWN',
        '\x1b[C': 'ARROW_RIGHT',
        '\x1b[D': 'ARROW_LEFT',
    }
    fd = sys.stdin.fileno()
    while True:
        try:
            # Set raw mode to capture every keypress
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not rlist:
                termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
                continue

            ch = sys.stdin.read(1)

            if ch == '\x1b':
                # Peek for escape sequence (arrow key = ESC [ X)
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist2:
                    seq = sys.stdin.read(2)
                    termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
                    full = ch + seq
                    mapped = ARROW_MAP.get(full)
                    if mapped:
                        key_q.put(mapped)
                    # else: unknown escape → ignore
                else:
                    termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)
                    # lone ESC → ignore
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

    # ── Constants ─────────────────────────────────────────────────────────────
    # Helipad delivery destinations (from city.sdf seed=42)
    DESTINATIONS = [
        (-45.0, -45.0, 10.8),
        (-45.0, -15.0, 27.7),
        (-30.0,  30.0, 27.5),
        (-15.0,   0.0, 10.3),
        (  0.0, -15.0, 38.4),
        (  0.0,  15.0, 19.4),
        ( 15.0, -15.0, 30.2),
        ( 30.0, -45.0, 35.1),
    ]

    TRUCK_START  = [0.0, -37.5, 1.5]   # truck centre (x, y, z)
    TRUCK_SPEED  = 3.0                  # metres per arrow keypress
    CRUISE_ALT   = 50.0                 # safe flying altitude above buildings
    PICKUP_HOVER = 5.0                  # hover height above truck when picking

    def __init__(self):
        super().__init__('teleop_node')

        # ── ROS services ──────────────────────────────────────────────────────
        self.set_client    = self.create_client(SetEntityState, '/set_entity_state')
        self.spawn_client  = self.create_client(SpawnEntity,    '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity,   '/delete_entity')

        self.get_logger().info('Waiting for /set_entity_state …')
        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  still waiting…')
        self.get_logger().info('Services ready.')

        # ── Truck state ───────────────────────────────────────────────────────
        self.truck_pos = list(self.TRUCK_START)

        # ── Drone state ───────────────────────────────────────────────────────
        tx, ty, tz = self.TRUCK_START
        # Drones start ABOVE the truck in a 2×2 cluster
        self.current_positions = {
            'drone1': [tx - 2.0, ty + 1.5, tz + 4.0],
            'drone2': [tx + 2.0, ty + 1.5, tz + 4.0],
            'drone3': [tx - 2.0, ty - 1.5, tz + 4.0],
            'drone4': [tx + 2.0, ty - 1.5, tz + 4.0],
        }
        self.drone_name = 'drone1'
        self.holding = None          # package held by the MANUAL drone

        # ── Batteries ─────────────────────────────────────────────────────────
        self.batteries = {f'drone{i}': 100.0 for i in range(1, 5)}

        # ── Mission / packages ────────────────────────────────────────────────
        self.packages = {}
        # package data structure:
        # {
        #   'dest':        (x, y, z),
        #   'color':       (r, g, b),
        #   'slot':        int,          # truck bed slot index
        #   'on_truck':    bool,         # physically sitting on truck
        #   'claimed_by':  str | None,   # drone_id that claimed it (before pickup)
        #   'delivered':   bool,
        # }

        # ── ACO / Auto ────────────────────────────────────────────────────────
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        try:
            from modules.aco_solver import ACOSolver
            self.aco_solver = ACOSolver()
        except ImportError as e:
            self.aco_solver = None
            self.get_logger().warn(f'ACOSolver missing ({e}). Auto mode disabled.')

        self.mode = 'manual'

        # Per-drone auto state machine
        self.drone_states   = {f'drone{i}': 'IDLE' for i in range(1, 5)}
        self.drone_targets  = {f'drone{i}': None   for i in range(1, 5)}
        self.drone_payloads = {f'drone{i}': None   for i in range(1, 5)}

        # Lock for thread-safe package claiming
        self._pkg_lock = threading.Lock()

        # ── Model tracking ────────────────────────────────────────────────────
        self.box_poses = {}
        self.model_sub = self.create_subscription(
            ModelStates, '/model_states', self._model_cb, 10)

        # ── Cameras ───────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.frames = {f'drone{i}': None for i in range(1, 5)}
        for i in range(1, 5):
            self.create_subscription(
                Image,
                f'/drone{i}/downward_camera/image_raw',
                lambda msg, d=f'drone{i}': self._image_cb(msg, d),
                10)
            cv2.namedWindow(f'Drone {i} View', cv2.WINDOW_NORMAL)
            cv2.resizeWindow(f'Drone {i} View', 320, 240)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _model_cb(self, msg):
        try:
            for i, name in enumerate(msg.name):
                if name.startswith('package_') or name == 'delivery_truck':
                    self.box_poses[name] = msg.pose[i]
        except Exception:
            pass

    def _image_cb(self, msg, drone_id):
        try:
            self.frames[drone_id] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge {drone_id}: {e}')

    # ── Camera HUD ────────────────────────────────────────────────────────────

    def show_cameras(self):
        key = -1
        for d_id, frame in self.frames.items():
            if frame is None:
                continue
            frame = frame.copy()
            h, w = frame.shape[:2]

            cr, cg, cb = DRONE_COLORS[d_id]
            border_color = (int(cb*255), int(cg*255), int(cr*255))  # BGR

            # Active drone border
            if d_id == self.drone_name:
                cv2.rectangle(frame, (0, 0), (w-1, h-1), (0, 255, 0), 4)
            else:
                cv2.rectangle(frame, (0, 0), (w-1, h-1), border_color, 2)

            # Crosshair
            cx, cy = w // 2, h // 2
            cv2.line(frame, (cx-10, cy), (cx+10, cy), (0, 200, 255), 1)
            cv2.line(frame, (cx, cy-10), (cx, cy+10), (0, 200, 255), 1)

            # HUD text
            pos   = self.current_positions[d_id]
            batt  = self.batteries[d_id]
            state = self.drone_states[d_id]
            payload = self.drone_payloads.get(d_id) or (
                self.holding if d_id == self.drone_name else None)
            pkg_str   = f' ▲{payload}' if payload else ''
            mode_str  = 'AUTO' if self.mode == 'auto' else 'MAN'
            hud = (f'[{mode_str}][{state[:6]}]'
                   f' ({pos[0]:.0f},{pos[1]:.0f},{pos[2]:.0f})'
                   f' {batt:.0f}%{pkg_str}')

            if batt < 20.0:
                bg, fg = (0, 0, 200), (255, 255, 255)
            else:
                bg, fg = (20, 20, 20), (180, 255, 180)

            cv2.rectangle(frame, (0, h-24), (w, h), bg, -1)
            cv2.putText(frame, hud, (4, h-7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, fg, 1)

            # Package colour swatch if carrying
            if payload and payload in self.packages:
                pr, pg, pb = self.packages[payload]['color']
                swatch = (int(pb*255), int(pg*255), int(pr*255))
                cv2.rectangle(frame, (w-20, h-24), (w, h), swatch, -1)

            cv2.imshow(f'Drone {d_id[-1]} View', frame)
            k = cv2.waitKey(1)
            if k != -1:
                key = k
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
        """Move an entity; silently ignores if not yet spawned."""
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x    = float(x)
        req.state.pose.position.y    = float(y)
        req.state.pose.position.z    = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame    = 'world'
        self.set_client.call_async(req)

    def _set_pose(self, x, y, z):
        """Move the manually-selected drone (and its held package if not delivered)."""
        self._set_entity_pose(self.drone_name, x, y, z)
        if self.holding:
            data = self.packages.get(self.holding, {})
            if not data.get('delivered'):
                self._set_entity_pose(self.holding, x, y, z - 0.6)

    def _set_pose_auto(self, d_id, x, y, z):
        """Move an auto drone and its payload package (if not delivered)."""
        self._set_entity_pose(d_id, x, y, z)
        payload = self.drone_payloads.get(d_id)
        if payload:
            data = self.packages.get(payload, {})
            if not data.get('delivered') and not data.get('on_truck'):
                self._set_entity_pose(payload, x, y, z - 0.6)

    # ── Battery drain ─────────────────────────────────────────────────────────

    def _drain(self, d_id, dist, has_payload=False):
        rate = 0.015 + (0.025 if has_payload else 0.0)
        self.batteries[d_id] = max(0.0, self.batteries[d_id] - dist * rate)

    # ── Manual drone movement ─────────────────────────────────────────────────

    def move(self, dx, dy, dz):
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        self._drain(self.drone_name, dist, bool(self.holding))
        pos = self.current_positions[self.drone_name]
        pos[0] += dx
        pos[1] += dy
        pos[2] += dz
        self._set_pose(pos[0], pos[1], pos[2])
        self.get_logger().info(
            f'{self.drone_name} → ({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})'
            f'  Batt:{self.batteries[self.drone_name]:.1f}%')

    def toggle_takeoff(self):
        z = self.current_positions[self.drone_name][2]
        target = 50.0 if z < 5.0 else 0.5
        self.move(0, 0, target - z)

    def switch_drone(self, n):
        self.drone_name = f'drone{n}'
        pos = self.current_positions[self.drone_name]
        self._set_pose(pos[0], pos[1], pos[2])
        self.get_logger().info(f'Selected {self.drone_name}')

    def update_held_object(self):
        """Keep held package glued to drone every tick."""
        if self.holding:
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

        # Move packages that are still physically sitting on the truck
        for pkg, data in self.packages.items():
            if data.get('on_truck') and not data.get('delivered'):
                sx, sy, sz = self._pkg_slot_pos(data['slot'])
                self._set_entity_pose(pkg, sx, sy, sz)

    def _pkg_slot_pos(self, slot):
        """Return world position for a package slot on the truck bed."""
        tx, ty, tz = self.truck_pos
        col = slot % 3   # 0,1,2
        row = slot // 3  # 0..4
        sx = tx - 2.0 + col * 1.3    # spread across truck bed (back half)
        sy = ty - 1.2 + row * 0.6
        sz = tz + 1.6                 # on top of truck bed
        return sx, sy, sz

    # ── Mission setup ─────────────────────────────────────────────────────────

    def setup_mission(self):
        self.get_logger().info('Setting up mission…')

        # ── Spawn delivery truck ──────────────────────────────────────────────
        truck_xml = (
            '<?xml version="1.0"?><sdf version="1.6">'
            '<model name="delivery_truck"><static>false</static>'
            '<link name="link"><gravity>0</gravity>'
            # Main cargo body
            '<visual name="body"><geometry><box><size>8.0 3.5 2.5</size></box></geometry>'
            '<material><ambient>1.0 0.45 0.0 1</ambient><diffuse>1.0 0.45 0.0 1</diffuse></material></visual>'
            '<collision name="c_body"><geometry><box><size>8.0 3.5 2.5</size></box></geometry></collision>'
            # Cab
            '<visual name="cab"><pose>3.8 0 1.4 0 0 0</pose><geometry><box><size>2.2 3.2 2.0</size></box></geometry>'
            '<material><ambient>0.85 0.35 0.0 1</ambient><diffuse>0.85 0.35 0.0 1</diffuse></material></visual>'
            # Windscreen
            '<visual name="ws"><pose>4.7 0 2.1 0.35 0 0</pose><geometry><box><size>0.1 2.8 1.3</size></box></geometry>'
            '<material><ambient>0.5 0.8 1.0 0.6</ambient><diffuse>0.5 0.8 1.0 0.6</diffuse></material></visual>'
            # Front bumper stripe
            '<visual name="stripe"><pose>4.95 0 0.0 0 0 0</pose><geometry><box><size>0.15 3.5 0.3</size></box></geometry>'
            '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            # Wheels (6 total)
            '<visual name="wfl"><pose>2.8 2.0 -0.75 1.5708 0 0</pose><geometry><cylinder><radius>0.7</radius><length>0.4</length></cylinder></geometry>'
            '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            '<visual name="wfr"><pose>2.8 -2.0 -0.75 1.5708 0 0</pose><geometry><cylinder><radius>0.7</radius><length>0.4</length></cylinder></geometry>'
            '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            '<visual name="wml"><pose>0.0 2.0 -0.75 1.5708 0 0</pose><geometry><cylinder><radius>0.7</radius><length>0.4</length></cylinder></geometry>'
            '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            '<visual name="wmr"><pose>0.0 -2.0 -0.75 1.5708 0 0</pose><geometry><cylinder><radius>0.7</radius><length>0.4</length></cylinder></geometry>'
            '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            '<visual name="wbl"><pose>-2.5 2.0 -0.75 1.5708 0 0</pose><geometry><cylinder><radius>0.7</radius><length>0.4</length></cylinder></geometry>'
            '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            '<visual name="wbr"><pose>-2.5 -2.0 -0.75 1.5708 0 0</pose><geometry><cylinder><radius>0.7</radius><length>0.4</length></cylinder></geometry>'
            '<material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>'
            '</link></model></sdf>'
        )
        self._spawn('delivery_truck',
                    truck_xml,
                    self.truck_pos[0], self.truck_pos[1], self.truck_pos[2])
        time.sleep(0.4)

        # ── Spawn drones above truck ──────────────────────────────────────────
        for d_id, pos in self.current_positions.items():
            cr, cg, cb = DRONE_COLORS[d_id]
            drone_xml = (
                f'<?xml version="1.0"?><sdf version="1.6">'
                f'<model name="{d_id}"><static>false</static>'
                f'<link name="base_link"><gravity>0</gravity>'
                f'<visual name="body"><geometry><box><size>0.5 0.5 0.12</size></box></geometry>'
                f'<material><ambient>{cr:.2f} {cg:.2f} {cb:.2f} 1</ambient>'
                f'<diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1</diffuse></material></visual>'
                f'<collision name="c"><geometry><box><size>0.5 0.5 0.12</size></box></geometry></collision>'
                # 4 propeller disks
                f'<visual name="p1"><pose>0.3 -0.3 0.06 0 0 0</pose>'
                f'<geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>'
                f'<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>'
                f'<visual name="p2"><pose>0.3 0.3 0.06 0 0 0</pose>'
                f'<geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>'
                f'<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>'
                f'<visual name="p3"><pose>-0.3 -0.3 0.06 0 0 0</pose>'
                f'<geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>'
                f'<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>'
                f'<visual name="p4"><pose>-0.3 0.3 0.06 0 0 0</pose>'
                f'<geometry><cylinder><radius>0.18</radius><length>0.02</length></cylinder></geometry>'
                f'<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>'
                f'</link></model></sdf>'
            )
            self._spawn(d_id, drone_xml, pos[0], pos[1], pos[2])
            time.sleep(0.1)

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

            pkg_xml = _box_sdf(pkg_name, 0.85, 0.85, 0.85,
                               r, g, b, 1.0, static=False, gravity=False)
            self._spawn(pkg_name, pkg_xml, sx, sy, sz)
            time.sleep(0.04)

        self.get_logger().info('Mission ready: truck + 4 drones + 15 packages spawned.')

    # ── Manual pick / drop ────────────────────────────────────────────────────

    def pick_object(self):
        """Manual: pick next available package from the truck."""
        if self.holding:
            self.get_logger().info('Already holding a package.')
            return

        pos = self.current_positions[self.drone_name]
        tx, ty = self.truck_pos[0], self.truck_pos[1]
        dist_to_truck = math.sqrt((pos[0]-tx)**2 + (pos[1]-ty)**2)

        if dist_to_truck > 7.0:
            self.get_logger().info(
                f'Too far from truck ({dist_to_truck:.1f} m). Fly closer.')
            return

        with self._pkg_lock:
            for pkg_name, data in self.packages.items():
                if data['delivered'] or not data['on_truck']:
                    continue
                if data['claimed_by'] is not None:
                    continue
                # Claim it
                data['claimed_by'] = self.drone_name
                data['on_truck']   = False
                self.holding       = pkg_name
                break

        if not self.holding:
            self.get_logger().info('No unclaimed packages on truck.')
            return

        dest = self.packages[self.holding]['dest']
        cr, cg, cb = self.packages[self.holding]['color']
        self.get_logger().info(
            f'Picked {self.holding} (RGB {cr:.1f},{cg:.1f},{cb:.1f}) '
            f'→ dest X:{dest[0]:.0f} Y:{dest[1]:.0f} Z:{dest[2]:.0f}')

        # Spawn destination marker
        marker_xml = _cylinder_sdf(
            f'marker_{self.holding}', 1.5, 20.0, cr, cg, cb, 0.6)
        self._spawn(f'marker_{self.holding}', marker_xml,
                    dest[0], dest[1], dest[2] + 10.0)

    def drop_object(self):
        if not self.holding:
            self.get_logger().info('Not holding anything.')
            return

        pkg = self.holding
        self.holding = None

        # Remove marker
        del_req = DeleteEntity.Request()
        del_req.name = f'marker_{pkg}'
        self.delete_client.call_async(del_req)

        dest = self.packages[pkg]['dest']
        pos  = self.current_positions[self.drone_name]
        dist = math.sqrt((pos[0]-dest[0])**2 + (pos[1]-dest[1])**2)

        if dist < 5.0:
            self.packages[pkg]['delivered']  = True
            self.packages[pkg]['claimed_by'] = None
            self.get_logger().info(f'✔ {pkg} delivered!')
        else:
            # Return to truck
            with self._pkg_lock:
                self.packages[pkg]['on_truck']   = True
                self.packages[pkg]['claimed_by'] = None
            # Physically move it back
            sx, sy, sz = self._pkg_slot_pos(self.packages[pkg]['slot'])
            self._set_entity_pose(pkg, sx, sy, sz)
            self.get_logger().info(f'{pkg} dropped away from dest — returned to truck.')

    # ── Auto mode ─────────────────────────────────────────────────────────────

    def toggle_mode(self):
        if self.mode == 'manual':
            self.mode = 'auto'
            self.get_logger().info('★ AUTO MODE activated — drones communicating via battery priority')
            for d_id in self.drone_states:
                self.drone_states[d_id] = 'IDLE'
        else:
            self.mode = 'manual'
            self.get_logger().info('★ MANUAL MODE activated')

    def _assign_packages(self):
        """
        Battery-priority package assignment.
        All IDLE drones "broadcast" their battery level; highest battery wins
        the next available package. This simulates inter-drone communication.
        """
        idle_drones = [
            (d_id, self.batteries[d_id])
            for d_id, st in self.drone_states.items()
            if st == 'IDLE' and self.batteries[d_id] > 5.0
        ]
        if not idle_drones:
            return

        # Sort by battery descending — highest charge gets first pick
        idle_drones.sort(key=lambda x: x[1], reverse=True)

        for d_id, batt in idle_drones:
            with self._pkg_lock:
                pkg = self._next_unclaimed()
                if pkg is None:
                    break
                self.packages[pkg]['claimed_by'] = d_id

            self.drone_payloads[d_id] = pkg
            tx, ty = self.truck_pos[0], self.truck_pos[1]
            self.drone_targets[d_id] = (tx, ty,
                                         self.truck_pos[2] + self.PICKUP_HOVER)
            pos = self.current_positions[d_id]
            self._draw_route(d_id, pos[0], pos[1], tx, ty)
            self.drone_states[d_id] = 'FLYING_TO_TRUCK'

            r, g, b = self.packages[pkg]['color']
            self.get_logger().info(
                f'{d_id} (batt {batt:.0f}%) → claimed {pkg} '
                f'(RGB {r:.1f},{g:.1f},{b:.1f})')

    def _next_unclaimed(self):
        """Return name of first on-truck, unclaimed, undelivered package."""
        for pkg, data in self.packages.items():
            if (data['on_truck'] and
                    not data['delivered'] and
                    data['claimed_by'] is None):
                return pkg
        return None

    def _draw_route(self, d_id, sx, sy, ex, ey):
        dx, dy = ex - sx, ey - sy
        dist   = math.sqrt(dx*dx + dy*dy)
        if dist < 0.5:
            return
        mx, my = sx + dx/2, sy + dy/2
        yaw    = math.atan2(dy, dx)
        qz, qw = math.sin(yaw/2), math.cos(yaw/2)
        cr, cg, cb = DRONE_COLORS[d_id]
        xml = _box_sdf(f'route_{d_id}', dist, 0.6, 0.15,
                       cr, cg, cb, 0.8, static=True)
        self._spawn(f'route_{d_id}', xml, mx, my, self.CRUISE_ALT, qz, qw)

    def _del_route(self, d_id):
        req = DeleteEntity.Request()
        req.name = f'route_{d_id}'
        self.delete_client.call_async(req)

    def auto_tick(self):
        """State machine tick for all autonomous drones."""
        # Step 1: Assign packages to waiting IDLE drones by battery priority
        self._assign_packages()

        # Step 2: Advance each drone's state
        for d_id in list(self.drone_states.keys()):
            if self.batteries[d_id] <= 0:
                continue

            state = self.drone_states[d_id]
            pos   = self.current_positions[d_id]

            # ── FLYING_TO_TRUCK ───────────────────────────────────────────────
            if state == 'FLYING_TO_TRUCK':
                tx, ty = self.truck_pos[0], self.truck_pos[1]
                dx = tx - pos[0]
                dy = ty - pos[1]
                horiz = math.sqrt(dx*dx + dy*dy)

                if pos[2] < self.CRUISE_ALT:
                    step = min(1.5, self.CRUISE_ALT - pos[2])
                    pos[2] += step
                    self._drain(d_id, step)
                    self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                elif horiz > 2.0:
                    step = min(1.5, horiz)
                    pos[0] += step * dx / horiz
                    pos[1] += step * dy / horiz
                    self._drain(d_id, step)
                    self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                else:
                    # Descend to pickup hover height
                    hover_z = self.truck_pos[2] + self.PICKUP_HOVER
                    if pos[2] > hover_z + 0.5:
                        step = min(1.2, pos[2] - hover_z)
                        pos[2] -= step
                        self._drain(d_id, step)
                        self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                    else:
                        self._del_route(d_id)
                        self.drone_states[d_id] = 'PICKING_UP'

            # ── PICKING_UP ────────────────────────────────────────────────────
            elif state == 'PICKING_UP':
                pkg = self.drone_payloads[d_id]
                if pkg and not self.packages[pkg]['delivered']:
                    # Mark package as no longer on truck (drone has grabbed it)
                    self.packages[pkg]['on_truck'] = False
                    # Snap it to drone belly
                    self._set_entity_pose(pkg, pos[0], pos[1], pos[2] - 0.6)

                    dest = self.packages[pkg]['dest']
                    self.drone_targets[d_id] = dest
                    self._draw_route(d_id, pos[0], pos[1], dest[0], dest[1])

                    # Destination marker in package colour
                    cr, cg, cb = self.packages[pkg]['color']
                    marker_xml = _cylinder_sdf(
                        f'marker_{pkg}', 1.5, 20.0, cr, cg, cb, 0.7)
                    self._spawn(f'marker_{pkg}', marker_xml,
                                dest[0], dest[1], dest[2] + 10.0)

                    self.get_logger().info(f'{d_id}: picked {pkg} → {dest}')
                    self.drone_states[d_id] = 'FLYING_TO_DROP'
                else:
                    # Package was already delivered by someone else
                    self.drone_payloads[d_id] = None
                    self.drone_states[d_id]   = 'IDLE'

            # ── FLYING_TO_DROP ────────────────────────────────────────────────
            elif state == 'FLYING_TO_DROP':
                target = self.drone_targets[d_id]
                dx = target[0] - pos[0]
                dy = target[1] - pos[1]
                horiz = math.sqrt(dx*dx + dy*dy)

                if pos[2] < self.CRUISE_ALT:
                    step = min(1.5, self.CRUISE_ALT - pos[2])
                    pos[2] += step
                    self._drain(d_id, step, True)
                    self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                elif horiz > 1.5:
                    step = min(1.5, horiz)
                    pos[0] += step * dx / horiz
                    pos[1] += step * dy / horiz
                    self._drain(d_id, step, True)
                    self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                else:
                    self._del_route(d_id)
                    self.drone_states[d_id] = 'DESCENDING_TO_DROP'

            # ── DESCENDING_TO_DROP ────────────────────────────────────────────
            elif state == 'DESCENDING_TO_DROP':
                target   = self.drone_targets[d_id]
                target_z = target[2] + 2.5
                if pos[2] > target_z + 0.5:
                    step = min(1.2, pos[2] - target_z)
                    pos[2] -= step
                    self._drain(d_id, step, True)
                    self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                else:
                    self.drone_states[d_id] = 'DROPPING'

            # ── DROPPING ──────────────────────────────────────────────────────
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

            # ── RETURNING_TO_TRUCK ────────────────────────────────────────────
            elif state == 'RETURNING_TO_TRUCK':
                tx, ty, tz = self.truck_pos
                dx = tx - pos[0]
                dy = ty - pos[1]
                horiz = math.sqrt(dx*dx + dy*dy)

                if pos[2] < self.CRUISE_ALT:
                    step = min(1.5, self.CRUISE_ALT - pos[2])
                    pos[2] += step
                    self._drain(d_id, step)
                    self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                elif horiz > 2.0:
                    step = min(1.5, horiz)
                    pos[0] += step * dx / horiz
                    pos[1] += step * dy / horiz
                    self._drain(d_id, step)
                    self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                else:
                    hover_z = tz + self.PICKUP_HOVER
                    if pos[2] > hover_z + 0.5:
                        step = min(1.2, pos[2] - hover_z)
                        pos[2] -= step
                        self._drain(d_id, step)
                        self._set_pose_auto(d_id, pos[0], pos[1], pos[2])
                    else:
                        self.drone_states[d_id] = 'IDLE'   # ready for next package


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    orig_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    teleop = TeleopNode()

    print(HELP_MSG)

    # ROS spin thread
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(teleop,), daemon=True)
    ros_thread.start()

    # Dedicated keyboard thread (handles regular keys + arrow sequences)
    key_q = queue.Queue()
    kb_thread = threading.Thread(
        target=keyboard_reader_thread,
        args=(orig_settings, key_q),
        daemon=True)
    kb_thread.start()

    try:
        teleop.setup_mission()

        while True:
            # Glue held package to manual drone
            teleop.update_held_object()

            # Advance auto state machine
            if teleop.mode == 'auto':
                teleop.auto_tick()

            # Show camera feeds; collect any CV2 keypress
            cv_key = teleop.show_cameras()
            if cv_key != -1:
                k = chr(cv_key).lower()
            else:
                k = None

            # Drain key queue (prefer terminal input)
            try:
                term_k = key_q.get_nowait()
            except queue.Empty:
                term_k = None

            # ── Arrow keys: truck control (ALWAYS active) ─────────────────────
            truck_step = TeleopNode.TRUCK_SPEED
            if term_k == 'ARROW_UP':
                teleop._move_truck( truck_step, 0)
                continue
            elif term_k == 'ARROW_DOWN':
                teleop._move_truck(-truck_step, 0)
                continue
            elif term_k == 'ARROW_LEFT':
                teleop._move_truck(0,  truck_step)
                continue
            elif term_k == 'ARROW_RIGHT':
                teleop._move_truck(0, -truck_step)
                continue

            # Resolve final character key (terminal wins over CV2)
            if term_k:
                k = term_k.lower() if len(term_k) == 1 else None

            if not k:
                continue

            # ── CTRL-C ─────────────────────────────────────────────────────────
            if k == '\x03':
                break

            # ── Mode toggle ────────────────────────────────────────────────────
            if k == 'm':
                teleop.toggle_mode()
                continue

            # ── Drone selection (both modes) ───────────────────────────────────
            if k in ('1', '2', '3', '4'):
                teleop.switch_drone(int(k))
                continue

            # ── Manual drone controls (blocked in auto mode) ───────────────────
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

    except Exception as exc:
        import traceback
        traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        teleop.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
