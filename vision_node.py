# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import Image
# pyrefly: ignore [missing-import]
from std_msgs.msg import String
# pyrefly: ignore [missing-import]
from cv_bridge import CvBridge
# pyrefly: ignore [missing-import]
import cv2
import json
import sys
import os
# pyrefly: ignore [missing-import]
import numpy as np

# Ensure modules are in path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from modules.perception import YOLOPerception, OpticalFlowTracker

class DroneVisionNode(Node):
    def __init__(self):
        super().__init__('drone_vision')
        
        # Subscribe to drone1's camera
        self.subscription = self.create_subscription(
            Image,
            '/drone1/downward_camera/image_raw',
            self.image_callback,
            10)
            
        self.bridge = CvBridge()
        
        self.get_logger().info('Initializing AI Perception & Semantic Comm Module...')
        self.perception = YOLOPerception(model_name='/mnt/d/Capstone/yolov8n.pt', conf_threshold=0.65)
        self.tracker = OpticalFlowTracker(drift_threshold=15.0)
        
        self.prev_gray = None
        self.prev_pts = None
        
        # Publishers for integration
        self.telemetry_pub = self.create_publisher(String, '/swarm_telemetry', 10)
        self.compressed_pub = self.create_publisher(Image, '/drone1/downward_camera/compressed_image', 10)
        
        self.get_logger().info('YOLOv8 + Semantic Communication Ready! Waiting for camera feed...')

    def image_callback(self, msg):
        try:
            # Convert ROS 2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        # 1. Perception & Mode Selection
        boxes, confidences, Sk = self.perception.detect_objects(cv_image)
        mode_decision = self.perception.select_compression_mode(Sk, raw_image=cv_image)
        
        # 2. Optical Flow Drift Tracking
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        drift_variance = 0.0
        if self.prev_gray is not None:
            self.prev_pts, st, drift_variance = self.tracker.track_lk_optical_flow(self.prev_gray, gray, self.prev_pts)
        else:
            self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.prev_gray = gray.copy()
        
        # 3. Publish Telemetry for AIDecisionEngine
        telemetry_msg = {
            'drone_id': 1,
            'confidence_Sk': float(Sk),
            'drift_variance': float(drift_variance),
            'payload_kb': float(mode_decision['payload_kb']),
            'comm_mode': mode_decision['mode']
        }
        pub_msg = String()
        pub_msg.data = json.dumps(telemetry_msg)
        self.telemetry_pub.publish(pub_msg)
        
        # Publish and show the Semantic Image
        comp_img = mode_decision['compressed_image']
        if comp_img is not None:
            comp_ros_img = self.bridge.cv2_to_imgmsg(comp_img, "bgr8")
            self.compressed_pub.publish(comp_ros_img)
            
            # Show output locally
            for box in boxes:
                bx, by, bw_box, bh_box = [int(v) for v in box]
                x1, y1 = max(0, bx - bw_box // 2), max(0, by - bh_box // 2)
                x2, y2 = bx + bw_box // 2, by + bh_box // 2
                cv2.rectangle(comp_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
            # Add telemetry overlay for presentation
            cv2.putText(comp_img, f"Sk (Conf): {Sk:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(comp_img, f"Mode: {mode_decision['mode']}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(comp_img, f"Payload: {mode_decision['payload_kb']:.1f} KB", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow("Drone 1 - Semantic Communication Feed", comp_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DroneVisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
