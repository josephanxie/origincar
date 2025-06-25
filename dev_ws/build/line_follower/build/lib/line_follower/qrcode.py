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
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()
        self.timer = self.create_timer(1, self.timer_callback)
        # self.publisher_ = self.create_publisher(Image, 'detected_image', 10)
        # self.signal = self.create_publisher()
        # self.stop_signal_sent = False
        self.status = True

    def timer_callback(self):
        self.status = not self.status

    def image_callback(self, msg):
        if self.status == True:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        a = pyzbar.decode(cv_image)

        self.get_logger().info(f"{a}")


        # is_ok, qr_res, qr_type, points = self.qr_decoder.detectAndDecode(cv_image)
        
        # self.get_logger().info(f"is_ok: {is_ok}")
        # self.get_logger().info(f"qr_res: {qr_res}")
        # self.get_logger().info(f"qr_type: {qr_type}")
        # self.get_logger().info(f"points: {points}")


        # if points is not None:
        #     n_points = len(points)
        #     for i in range(n_points):
        #         next_point = i + 1 if i < n_points - 1 else 0
        #         cv2.line(cv_image, tuple(points[i][0]), tuple(points[next_point][0]), (0, 255, 0), 3)

        #     if data:
        #         occupation = self.calculate_occupation(points)
        #         # cv2.putText(cv_image, f'Distance: {distance:.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        #         if occupation > 0.8 and not self.stop_signal_sent:  
        #             self.send_stop_signal()
        #             self.stop_signal_sent = True
            
        # self.publisher_.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
