# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import Image
# pyrefly: ignore [missing-import]
from cv_bridge import CvBridge
# pyrefly: ignore [missing-import]
import cv2
# pyrefly: ignore [missing-import]
from ultralytics import YOLO

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
        
        # Load the YOLOv8 model
        self.get_logger().info('Loading YOLOv8 model...')
        self.model = YOLO('/mnt/d/Capstone/yolov8n.pt')
        self.get_logger().info('YOLOv8 ready! Waiting for camera feed...')

    def image_callback(self, msg):
        try:
            # Convert ROS 2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        # Run YOLOv8 inference
        results = self.model(cv_image, verbose=False)
        
        # Plot the bounding boxes on the image
        annotated_frame = results[0].plot()
        
        # Display the live feed natively on Windows via WSLg
        cv2.imshow("Drone 1 - Downward Camera (YOLOv8)", annotated_frame)
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
