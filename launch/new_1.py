import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
import numpy as np


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, '/my_compressed_image', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Edge detection
        edges = cv2.Canny(gray_image, 100, 200)

        # Publish compressed image
        msg = CompressedImage()
        msg.header = msg.header
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', edges)[1]).tobytes()
	    # msg.data = np.array(cv2.imencode('.jpg', edges)[1]).tobytes()
        
        self.compressed_image_pub.publish(msg)

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
        
        cv2.imshow("edges", edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
