import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
import sys
import termios
import tty
import select
import threading

msg = """
MANUAL DRONE CONTROL
---------------------------
Move around:
   w
a  s  d

Up/Down:
q : Up (+Z)
e : Down (-Z)

CTRL-C to quit
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state service...')
        
        # Drone starting position
        self.x = -30.0
        self.y = 30.0
        self.z = 50.0
        
    def move(self, dx, dy, dz):
        self.x += dx
        self.y += dy
        self.z += dz
        
        req = SetEntityState.Request()
        req.state.name = 'drone1'
        req.state.pose.position.x = float(self.x)
        req.state.pose.position.y = float(self.y)
        req.state.pose.position.z = float(self.z)
        self.client.call_async(req)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    
    print(msg)
    
    # Spin node in background
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    
    try:
        while True:
            key = get_key(settings)
            step = 1.0
            
            if key == 'w':
                node.move(step, 0, 0)
            elif key == 's':
                node.move(-step, 0, 0)
            elif key == 'a':
                node.move(0, step, 0)
            elif key == 'd':
                node.move(0, -step, 0)
            elif key == 'q':
                node.move(0, 0, step)
            elif key == 'e':
                node.move(0, 0, -step)
            elif key == '\x03': # CTRL-C
                break
                
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
