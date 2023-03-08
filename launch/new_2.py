import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
import numpy as np


class MyImageProcessingNode(Node):
    def __init__(self):
        super().__init__('my_image_processing_node')

        # Create a CvBridge object to convert ROS messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the /camera/image_raw/compressed topic
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)

        # Create a publisher for the compressed image
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, '/my_compressed_image', 10)

        # Create a publisher for the robot's velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

    def image_callback(self, msg):
        # Decode the compressed image to an OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Edge detection
        edges = cv2.Canny(gray_image, 100, 200)

        # Create a CompressedImage message for the processed image
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(cv2.imencode('.jpg', edges)[1]).tobytes()

        # Publish the compressed image
        self.compressed_image_pub.publish(compressed_msg)

        # Move robot based on edge detection
        if np.sum(edges[0:10,:]) > 100:
            # Edge detected on left side, turn right
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = -0.5
            self.cmd_vel_pub.publish(twist)
        elif np.sum(edges[-10:,:]) > 100:
            # Edge detected on right side, turn left
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
        else:
            # No edge detected, move forward
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
        
        # Show the edge detection result in a window
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MyImageProcessingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()