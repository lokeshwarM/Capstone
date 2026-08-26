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
        self.box_pose = None

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
            if 'box_payload' in msg.name:
                idx = msg.name.index('box_payload')
                self.box_pose = msg.pose[idx]
        except Exception:
            pass

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
        pos = self.current_positions[self.drone_name]
        pos[0] += dx
        pos[1] += dy
        pos[2] += dz
        self._set_pose(pos[0], pos[1], pos[2])
        self.get_logger().info(f'{self.drone_name} -> ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

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
    # Object handling
    # ---------------------------------------------------------------------
    def spawn_box(self):
        # A bright green box (1x1x1) so it is very visible on the gray floor
        xml = """<?xml version="1.0" ?><sdf version="1.6"><model name="box"><static>false</static><link name="link"><visual name="visual"><geometry><box><size>1 1 1</size></box></geometry><material><ambient>0 1 0 1</ambient></material></visual><collision name="collision"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></sdf>"""
        
        # Try to delete the old box first so it doesn't get stuck in previous spawn locations
        del_req = DeleteEntity.Request()
        del_req.name = 'box_payload'
        self.delete_client.call_async(del_req)
        
        # Give it a tiny bit of time to delete before respawning
        def delayed_spawn():
            time.sleep(0.5)
            req = SpawnEntity.Request()
            req.name = 'box_payload'
            req.xml = xml
            # Spawn it right in front of drone1's starting position so you can see it!
            req.initial_pose.position.x = -7.5
            req.initial_pose.position.y = -5.0
            req.initial_pose.position.z = 1.0
            req.reference_frame = 'world'
            self.spawn_client.call_async(req)
            self.get_logger().info(f'Spawned new box_payload right in front of drone 1!')
            
        threading.Thread(target=delayed_spawn).start()

    def pick_object(self):
        if self.holding:
            self.get_logger().info('Already holding an object')
            return
            
        # We need to know where the box is to pick it up!
        if not hasattr(self, 'box_pose') or self.box_pose is None:
            self.get_logger().info('Cannot find box in the world. Is it spawned?')
            return
            
        # Calculate distance between drone and box
        drone_pos = self.current_positions[self.drone_name]
        dx = drone_pos[0] - self.box_pose.position.x
        dy = drone_pos[1] - self.box_pose.position.y
        dz = drone_pos[2] - self.box_pose.position.z
        
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # If the drone is within 3.5 meters of the box, it can grab it!
        if dist > 3.5:
            self.get_logger().info(f'Too far to pick up! Distance: {dist:.1f}m. Hover closer to the box.')
            return
            
        self.holding = 'box_payload'
        self.get_logger().info(f'Successfully picked up box_payload!')

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
