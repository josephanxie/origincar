import rclpy
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().info("start line follower.")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(Image, '/camera/processed_image', 10)
        self.twist = Twist()

def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([10, 70, 30])
    upper_yellow = np.array([255, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    h, w, d = image.shape
    search_top = int(h / 2)
    search_bot = int(h / 2 + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

        # 基于检测到的目标中心点，计算机器人的控制参数
        err = cx - w / 2
        self.twist.linear.x = 0.1
        self.twist.angular.z = -float(err) / 400
        self.cmd_vel_pub.publish(self.twist)
        self.pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    try:
        follower = Follower()
        rclpy.spin(follower)
    finally:
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()