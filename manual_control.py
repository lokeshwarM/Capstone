import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState, SpawnEntity, DeleteEntity
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import termios
import tty
import select
import threading
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
        
        while not self.set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')
            
        # Hardcode initial spawn positions so it doesn't teleport to 0,0,0
        self.spawn_positions = {
            'drone1': [-7.5, -7.5, 0.5],
            'drone2': [ 7.5, -7.5, 0.5],
            'drone3': [-7.5,  7.5, 0.5],
            'drone4': [ 7.5,  7.5, 0.5]
        }
        
        self.drone_name = 'drone1'
        self.x = self.spawn_positions['drone1'][0]
        self.y = self.spawn_positions['drone1'][1]
        self.z = self.spawn_positions['drone1'][2]
        self.holding = None

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

    def image_cb(self, msg, drone_id):
        try:
            self.frames[drone_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            pass

    def show_cameras(self):
        cv_key = -1
        for i in range(1, 5):
            drone_id = f'drone{i}'
            if self.frames[drone_id] is not None:
                frame = self.frames[drone_id].copy()
                if drone_id == self.drone_name:
                    cv2.rectangle(frame, (0,0), (frame.shape[1]-1, frame.shape[0]-1), (0,255,0), 10)
                cv2.imshow(f'Drone {i} View', frame)
        # Process OpenCV GUI events and get key if window is focused
        k = cv2.waitKey(10)
        if k != -1:
            cv_key = k & 0xFF
        return cv_key

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
        self.x += dx
        self.y += dy
        self.z += dz
        self._set_pose(self.x, self.y, self.z)
        self.get_logger().info(f'{self.drone_name} -> ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})')

    def toggle_takeoff(self):
        target = 50.0 if self.z < 5.0 else 0.5
        self.move(0, 0, target - self.z)
        self.get_logger().info(f'{self.drone_name} {"taking off" if target > 5.0 else "landing"}')

    # ---------------------------------------------------------------------
    # Drone selection
    # ---------------------------------------------------------------------
    def switch_drone(self, number):
        self.drone_name = f'drone{number}'
        self.get_logger().info(f'Switched control to {self.drone_name}')
        self.x = self.spawn_positions[self.drone_name][0]
        self.y = self.spawn_positions[self.drone_name][1]
        self.z = self.spawn_positions[self.drone_name][2]
        self._set_pose(self.x, self.y, self.z)

    # ---------------------------------------------------------------------
    # Object handling
    # ---------------------------------------------------------------------
    def spawn_box(self):
        xml = """<?xml version="1.0" ?><sdf version="1.6"><model name="box"><static>false</static><link name="link"><visual name="visual"><geometry><box><size>1 1 1</size></box></geometry><material><ambient>1 0 0 1</ambient></material></visual><collision name="collision"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></sdf>"""
        req = SpawnEntity.Request()
        req.name = 'box_payload'
        req.xml = xml
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 1.0
        req.reference_frame = 'world'
        self.spawn_client.call_async(req)
        self.get_logger().info(f'Spawned box_payload at origin (0, 0, 1)')

    def pick_object(self):
        if self.holding:
            self.get_logger().info('Already holding an object')
            return
        if self.z > 2.0:
            self.get_logger().info('Too high to pick object! Land first.')
            return
        self.holding = 'box_payload'
        self.get_logger().info(f'Picked up box_payload')

    def drop_object(self):
        if not self.holding:
            self.get_logger().info('Not holding any object')
            return
        self.holding = None
        self.get_logger().info(f'Dropped payload')


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
        teleop.spawn_box()
        
        while True:
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
                
            step = 2.0
            
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
