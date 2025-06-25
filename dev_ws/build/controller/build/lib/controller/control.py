import rclpy
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32



class Control(Node):
    def __init__(self):
        super().__init__('control')
        self.get_logger().info("Control node started.")
        self.control_sub = self.create_subscription(Int32, '/sign4return', self.main, 10)
        # self.image_sub = self.create_subscription(Image, '/image', self.callback, 10)
        # self.pub = self.create_publisher(Image, '/camera/processed_image', 10)
        # self.twist = Twist()

    def main(self, msg):
        self.get_logger().info(type(msg))



def main(args=None):
    rclpy.init(args=args)
    
    controller = Control()
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()