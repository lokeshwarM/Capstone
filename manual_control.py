# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from gazebo_msgs.srv import SetEntityState, GetEntityState, SpawnEntity, DeleteEntity
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import Image
# pyrefly: ignore [missing-import]
from cv_bridge import CvBridge
import sys
import termios
import tty
import select
import threading
# pyrefly: ignore [missing-import]
import cv2
import math
import time

# -----------------------------------------------------------------------------
# Helper constants & utilities
# -----------------------------------------------------------------------------

msg = """
MANUAL DRONE CONTROL
---------------------------
Select drone: press 1-4 (Drone1-Drone4)
Movement keys:
   w - forward  (+X)
   s - backward (-X)
   a - left     (+Y)
   d - right    (-Y)
Up/Down:
   q - up (+Z)
   e - down (-Z)
Take-off / Land:
   t - toggle between hover (Z≈50) and land (Z≈0.5)
Object handling:
   p - pick nearest object (cube)
   x - drop currently held object
CTRL-C - quit

Note: Make sure this terminal OR one of the camera windows is clicked/focused when pressing keys!
"""

# -----------------------------------------------------------------------------
# Teleoperation node (movement + object handling + camera viewing)
# -----------------------------------------------------------------------------

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # --- SERVICES ---
        self.set_client = self.create_client(SetEntityState, '/set_entity_state')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')
            
        # Track current positions for all drones so they don't teleport when switching
        self.current_positions = {
            'drone1': [-7.5, -7.5, 0.5],
            'drone2': [ 7.5, -7.5, 0.5],
            'drone3': [-7.5,  7.5, 0.5],
            'drone4': [ 7.5,  7.5, 0.5]
        }
        
        self.drone_name = 'drone1'
        self.holding = None
        self.box_poses = {}
        
        # --- BATTERY TRACKING ---
        self.batteries = {'drone1': 100.0, 'drone2': 100.0, 'drone3': 100.0, 'drone4': 100.0}

        # --- MODEL TRACKING ---
        # Must import inside or at top level. Let's do it here for safety.
        # pyrefly: ignore [missing-import]
        from gazebo_msgs.msg import ModelStates
        self.model_sub = self.create_subscription(ModelStates, '/model_states', self.model_cb, 10)

        # --- CAMERAS ---
        self.bridge = CvBridge()
        self.frames = { 'drone1': None, 'drone2': None, 'drone3': None, 'drone4': None }
        
        self.sub1 = self.create_subscription(Image, '/drone1/downward_camera/image_raw', lambda msg: self.image_cb(msg, 'drone1'), 10)
        self.sub2 = self.create_subscription(Image, '/drone2/downward_camera/image_raw', lambda msg: self.image_cb(msg, 'drone2'), 10)
        self.sub3 = self.create_subscription(Image, '/drone3/downward_camera/image_raw', lambda msg: self.image_cb(msg, 'drone3'), 10)
        self.sub4 = self.create_subscription(Image, '/drone4/downward_camera/image_raw', lambda msg: self.image_cb(msg, 'drone4'), 10)
        
        for i in range(1, 5):
            window_name = f'Drone {i} View'
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 320, 240)

    def model_cb(self, msg):
        try:
            for i, name in enumerate(msg.name):
                if name.startswith('package_'):
                    self.box_poses[name] = msg.pose[i]
        except Exception:
            pass

    def image_cb(self, msg, drone_id):
        try:
            self.frames[drone_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')

    def show_cameras(self):
        key = -1
        for d_id, frame in self.frames.items():
            if frame is not None:
                frame = frame.copy()
                # Highlight active drone
                if d_id == self.drone_name:
                    cv2.rectangle(frame, (0, 0), (frame.shape[1]-1, frame.shape[0]-1), (0, 255, 0), 3)
                
                # Get center pixel color for fun
                h, w = frame.shape[:2]
                
                # Add simple crosshair
                cv2.line(frame, (w//2-5, h//2), (w//2+5, h//2), (255, 0, 0), 1)
                cv2.line(frame, (w//2, h//2-5), (w//2, h//2+5), (255, 0, 0), 1)
                
                # Put coordinate & battery text at the bottom
                pos = self.current_positions[d_id]
                batt = self.batteries[d_id]
                text = f"(x={int(pos[0])},y={int(pos[1])}) | Batt: {batt:.1f}%"
                
                # Turn background red if battery is very low (< 20%)
                bg_color = (0, 0, 255) if batt < 20.0 else (255, 255, 255)
                text_color = (255, 255, 255) if batt < 20.0 else (0, 0, 0)
                
                cv2.rectangle(frame, (0, h-20), (w, h), bg_color, -1)
                cv2.putText(frame, text, (5, h-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1)
                
                cv2.imshow(f'{d_id.capitalize()} View', frame)
                k = cv2.waitKey(1)
                if k != -1:
                    key = k
        return key

    # ---------------------------------------------------------------------
    # Pose helpers
    # ---------------------------------------------------------------------
    def _set_pose(self, x, y, z):
        req = SetEntityState.Request()
        req.state.name = self.drone_name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self.set_client.call_async(req)
        
        if self.holding:
            req_obj = SetEntityState.Request()
            req_obj.state.name = self.holding
            req_obj.state.pose.position.x = float(x)
            req_obj.state.pose.position.y = float(y)
            req_obj.state.pose.position.z = float(z - 0.5)
            req_obj.state.reference_frame = 'world'
            self.set_client.call_async(req_obj)

    # ---------------------------------------------------------------------
    # Movement
    # ---------------------------------------------------------------------
    def move(self, dx, dy, dz):
        # Calculate distance for battery drain
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Apply battery drain: 0.1% per meter, plus 0.2% penalty if carrying a payload
        drain_rate = 0.1
        if self.holding:
            drain_rate += 0.2
        
        self.batteries[self.drone_name] = max(0.0, self.batteries[self.drone_name] - (dist * drain_rate))
        
        pos = self.current_positions[self.drone_name]
        pos[0] += dx
        pos[1] += dy
        pos[2] += dz
        self._set_pose(pos[0], pos[1], pos[2])
        self.get_logger().info(f'{self.drone_name} -> ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | Battery: {self.batteries[self.drone_name]:.1f}%')

    def toggle_takeoff(self):
        z = self.current_positions[self.drone_name][2]
        target = 50.0 if z < 5.0 else 0.5
        self.move(0, 0, target - z)
        self.get_logger().info(f'{self.drone_name} {"taking off" if target > 5.0 else "landing"}')
        
    def update_held_object(self):
        if self.holding:
            pos = self.current_positions[self.drone_name]
            req_obj = SetEntityState.Request()
            req_obj.state.name = self.holding
            req_obj.state.pose.position.x = float(pos[0])
            req_obj.state.pose.position.y = float(pos[1])
            req_obj.state.pose.position.z = float(pos[2] - 0.5)
            # Reset velocities to prevent gravity accumulation
            req_obj.state.twist.linear.x = 0.0
            req_obj.state.twist.linear.y = 0.0
            req_obj.state.twist.linear.z = 0.0
            req_obj.state.reference_frame = 'world'
            self.set_client.call_async(req_obj)

    # ---------------------------------------------------------------------
    # Drone selection
    # ---------------------------------------------------------------------
    def switch_drone(self, number):
        self.drone_name = f'drone{number}'
        self.get_logger().info(f'Switched control to {self.drone_name}')
        # DO NOT reset coordinates. Just fetch the current position we've been tracking!
        pos = self.current_positions[self.drone_name]
        self._set_pose(pos[0], pos[1], pos[2])

    # ---------------------------------------------------------------------
    # Multi-Object Mission & Delivery Handling
    # ---------------------------------------------------------------------
    def setup_mission(self):
        # 7 Helipad destinations generated from city.sdf
        self.destinations = [
            (-45.0, 15.0, 25.26), (-30.0, -45.0, 26.44), (-30.0, -30.0, 34.38),
            (-30.0, 15.0, 20.19), (-15.0, 15.0, 11.47), (-15.0, 30.0, 12.49),
            (30.0, 0.0, 12.82)
        ]
        
        self.packages = {}
        # Spawn 15 packages scattered around the origin
        for i in range(1, 16):
            pkg_name = f'package_{i}'
            # Random ground coordinate around center (-20 to 20)
            import random
            px = random.uniform(-20, 20)
            py = random.uniform(-20, 20)
            pz = 1.0
            
            # Assign a random destination
            dest = random.choice(self.destinations)
            self.packages[pkg_name] = {'start': (px, py, pz), 'dest': dest, 'delivered': False}
            
            xml = f"""<?xml version="1.0" ?><sdf version="1.6"><model name="{pkg_name}"><static>false</static><link name="link"><visual name="visual"><geometry><box><size>0.8 0.8 0.8</size></box></geometry><material><ambient>0 1 0 1</ambient></material></visual><collision name="collision"><geometry><box><size>0.8 0.8 0.8</size></box></geometry></collision></link></model></sdf>"""
            
            # Delete old if exists
            del_req = DeleteEntity.Request()
            del_req.name = pkg_name
            self.delete_client.call_async(del_req)
            
            # Spawn new
            req = SpawnEntity.Request()
            req.name = pkg_name
            req.xml = xml
            req.initial_pose.position.x = float(px)
            req.initial_pose.position.y = float(py)
            req.initial_pose.position.z = float(pz)
            req.reference_frame = 'world'
            # Small delay to ensure delete finishes
            time.sleep(0.05)
            self.spawn_client.call_async(req)
            
        self.get_logger().info('Mission setup complete: 15 packages spawned.')

    def pick_object(self):
        if self.holding:
            self.get_logger().info('Already holding an object')
            return
            
        drone_pos = self.current_positions[self.drone_name]
        
        # Find the closest package
        closest_pkg = None
        min_dist = float('inf')
        
        for pkg_name, data in self.packages.items():
            if data['delivered']: continue
            
            # Use tracked box_poses if available from model_states, else fallback to start pos
            if hasattr(self, 'box_poses') and pkg_name in self.box_poses:
                b_pos = self.box_poses[pkg_name]
                bx, by, bz = b_pos.position.x, b_pos.position.y, b_pos.position.z
            else:
                bx, by, bz = data['start']
                
            dx = drone_pos[0] - bx
            dy = drone_pos[1] - by
            dz = drone_pos[2] - bz
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist < min_dist:
                min_dist = dist
                closest_pkg = pkg_name
                
        if closest_pkg and min_dist <= 3.5:
            self.holding = closest_pkg
            dest = self.packages[closest_pkg]['dest']
            self.get_logger().info(f'Picked up {closest_pkg}! Deliver to X:{dest[0]:.0f}, Y:{dest[1]:.0f}, Z:{dest[2]:.0f}')
            
            # SPAWN A VISUAL MARKER AT THE DESTINATION
            marker_name = f'marker_{closest_pkg}'
            # A tall, semi-transparent red cylinder to mark the drop zone
            marker_xml = f"""<?xml version="1.0" ?><sdf version="1.6"><model name="{marker_name}"><static>true</static><link name="link"><visual name="visual"><geometry><cylinder><radius>1.5</radius><length>20.0</length></cylinder></geometry><material><ambient>1 0 0 0.5</ambient><diffuse>1 0 0 0.5</diffuse></material></visual></link></model></sdf>"""
            
            req = SpawnEntity.Request()
            req.name = marker_name
            req.xml = marker_xml
            req.initial_pose.position.x = float(dest[0])
            req.initial_pose.position.y = float(dest[1])
            req.initial_pose.position.z = float(dest[2] + 10.0) # Center the 20m cylinder on the helipad
            req.reference_frame = 'world'
            self.spawn_client.call_async(req)
            
        else:
            self.get_logger().info('No packages within 3.5m to pick up.')

    def drop_object(self):
        if not self.holding:
            self.get_logger().info('Not holding any object')
            return
            
        pkg = self.holding
        self.holding = None
        
        # REMOVE THE VISUAL MARKER
        marker_name = f'marker_{pkg}'
        del_marker_req = DeleteEntity.Request()
        del_marker_req.name = marker_name
        self.delete_client.call_async(del_marker_req)
        
        # Check if dropped at destination
        dest = self.packages[pkg]['dest']
        drone_pos = self.current_positions[self.drone_name]
        
        dx = drone_pos[0] - dest[0]
        dy = drone_pos[1] - dest[1]
        
        # If within 5 meters horizontally of the helipad
        if math.sqrt(dx*dx + dy*dy) < 5.0:
            self.packages[pkg]['delivered'] = True
            self.get_logger().info(f'SUCCESS! {pkg} delivered successfully!')
        else:
            self.get_logger().info(f'Dropped {pkg}. Not at destination (X:{dest[0]:.0f}, Y:{dest[1]:.0f}).')

# -----------------------------------------------------------------------------
# Keyboard utilities
# -----------------------------------------------------------------------------

def get_key_terminal(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# -----------------------------------------------------------------------------
# Main entry point
# -----------------------------------------------------------------------------

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    teleop = TeleopNode()
    
    print(msg)
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(teleop,), daemon=True)
    ros_thread.start()
    
    try:
        teleop.setup_mission()
        
        while True:
            # Continuously update the held object's position to prevent gravity from pulling it down
            teleop.update_held_object()
            
            # 1. Get key from OpenCV windows (if focused)
            cv_k = teleop.show_cameras()
            
            # 2. Get key from terminal (if focused)
            term_k = get_key_terminal(settings)
            
            key = ''
            if cv_k != -1:
                key = chr(cv_k).lower()
            elif term_k:
                key = term_k.lower()
                
            if not key:
                continue
                
            step = 1.5
            
            if key in ['1', '2', '3', '4']:
                teleop.switch_drone(int(key))
            elif key == 'w':
                teleop.move(step, 0, 0)
            elif key == 's':
                teleop.move(-step, 0, 0)
            elif key == 'a':
                teleop.move(0, step, 0)
            elif key == 'd':
                teleop.move(0, -step, 0)
            elif key == 'q':
                teleop.move(0, 0, step)
            elif key == 'e':
                teleop.move(0, 0, -step)
            elif key == 't':
                teleop.toggle_takeoff()
            elif key == 'p':
                teleop.pick_object()
            elif key == 'x':
                teleop.drop_object()
            elif key == '\x03': # CTRL-C
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
