from pyzbar import pyzbar
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector_node')
        self.qrcode_sub = self.create_subscription(Image, '/image_fox', self.qrcode, 1)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1, self.timer_callback)
        self.control_pub = self.create_publisher(Int32, '/update_status', 10)
        self.qrcode_sub = self.create_subscription(Image, '/image_fox', self.qrcode, 10)

        self.twist = Twist()
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.qr_result = 0

    def qrcode(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # img = img[:240, :320, :]
        # img = cv2.resize(img, (640*4, 480*4), interpolation=cv2.INTER_AREA)
        result = pyzbar.decode(img)
        if len(result) == 0:
            return False
        self.qr_result = result[0].data.decode()
        if self.qr_result == "AntiClockWise":
            self.qr_result = 4
        elif self.qr_result == "ClockWise":
            self.qr_result = 3
        else:
            raise ValueError("警告警告, 二维码内容不对")
        self.get_logger().info(f"已识别到二维码内容为: {result}")

        time.sleep(0.1)
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

        self.control_pub.publish(Int32(data=2))  # 到达二维码区域
        time.sleep(0.5)
        self.get_logger().info(f"qr_result:{self.qr_result} ")
        self.control_pub.publish(Int32(data=self.qr_result))  # 二维码内容

        return True


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
