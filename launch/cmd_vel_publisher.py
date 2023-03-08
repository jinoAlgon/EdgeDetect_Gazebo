import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class EdgeFollower(Node):
    def __init__(self):
        super().__init__('edge_follower')
        self.bridge = CvBridge()

        # Subscribe to the compressed image topic
        self.image_sub = self.create_subscription(
            Image, 'video_frames', self.image_callback, 10)

        # Create a publisher for the robot's velocity command topic
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Initialize the last error to zero
        self.last_error = 0


    def image_callback(self, msg):
        # Convert the compressed image to a BGR image
        np_arr = np.frombuffer(msg.data, np.uint8)
        bgr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert the BGR image to a grayscale image
        gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

        # Apply edge detection to the grayscale image
        edges = cv2.Canny(gray_image, 100, 200)

        # Find the contours in the edges image
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Compute the error as the distance from the center
        height, width = gray_image.shape
        cx, cy = width/2, height/2
        error = 0
        for cnt in contours:
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(bgr_image, (cx, cy), 4, (0, 0, 255), -1)
                error = cx - width/2

        # Compute the control signal based on the error
        Kp = 0.5
        Kd = 0.1
        derivative = error - self.last_error
        u = Kp*error + Kd*derivative

        # Update the last error
        self.last_error = error

        # Move the robot based on the control signal
        twist = Twist()
        if error == 0:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
        elif error > 0:
            twist.linear.x = 0.1
            twist.angular.z = -u/100
        else:
            twist.linear.x = 0.1
            twist.angular.z = -u/100

        # Publish the control signal
        self.vel_pub.publish(twist)

        # Display the black and white frame
        cv2.imshow("edges", edges)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    edge_follower = EdgeFollower()

    rclpy.spin(edge_follower)

    edge_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
