# pyrefly: ignore [missing-import]
# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from gazebo_msgs.srv import SetEntityState
import time
import threading

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.client = self.create_client(SetEntityState, '/set_entity_state')

    def set_pose(self, drone_name, x, y, z):
        req = SetEntityState.Request()
        req.state.name = drone_name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        
        # Fire and forget for smooth animation
        self.client.call_async(req)

def interpolate(start, end, progress):
    return start + (end - start) * progress

def fly_drone(node, drone_name, start_pos, end_pos, duration=5.0):
    hz = 30
    steps = int(duration * hz)
    for i in range(steps):
        progress = i / float(steps)
        x = interpolate(start_pos[0], end_pos[0], progress)
        y = interpolate(start_pos[1], end_pos[1], progress)
        z = interpolate(start_pos[2], end_pos[2], progress)
        node.set_pose(drone_name, x, y, z)
        time.sleep(1.0 / hz)
    node.set_pose(drone_name, end_pos[0], end_pos[1], end_pos[2])

def mission_thread(node):
    print("=== WAITING FOR GAZEBO CONNECTION ===")
    while not node.client.wait_for_service(timeout_sec=1.0):
        print("Waiting for /set_entity_state service...")
    print("=== CONNECTED TO GAZEBO! STARTING AUTONOMOUS DELIVERY MISSION ===")
    
    # Starting positions based on our launch file
    starts = {
        'drone1': [-7.5, -7.5, 0.5],
        'drone2': [7.5, -7.5, 0.5],
        'drone3': [-7.5, 7.5, 0.5],
        'drone4': [7.5, 7.5, 0.5],
    }

    # Step 1: Takeoff to 50m (above all buildings)
    print("--> 1. Initiating Takeoff to 50m Clearance Altitude...")
    threads = []
    for drone in starts.keys():
        end = [starts[drone][0], starts[drone][1], 50.0]
        t = threading.Thread(target=fly_drone, args=(node, drone, starts[drone], end, 4.0))
        t.start()
        threads.append(t)
    for t in threads: t.join()

    # Step 2: Converge on the center for package dropoff
    print("--> 2. Navigating to Central Dropzone...")
    threads = []
    for drone in starts.keys():
        start = [starts[drone][0], starts[drone][1], 50.0]
        end = [0.0, 0.0, 50.0]
        t = threading.Thread(target=fly_drone, args=(node, drone, start, end, 6.0))
        t.start()
        threads.append(t)
    for t in threads: t.join()

    # Step 3: Lowering to 40m to look for helipads
    print("--> 3. Descending to 40m Search Altitude...")
    threads = []
    for drone in starts.keys():
        t = threading.Thread(target=fly_drone, args=(node, drone, [0,0,50], [0,0,40], 3.0))
        t.start()
        threads.append(t)
    for t in threads: t.join()

    print("=== MISSION REACHED WAYPOINT. WAITING FOR VISION SYSTEM ===")

def main():
    rclpy.init()
    node = MissionController()
    
    # Run the mission in a separate thread so ROS 2 can spin in the main thread
    t = threading.Thread(target=mission_thread, args=(node,))
    t.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
