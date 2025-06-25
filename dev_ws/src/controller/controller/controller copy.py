import rclpy
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from origincar_msg.msg import Sign


class Control(Node):
    def __init__(self):
        super().__init__('control')
        self.get_logger().info("Control node started.")
        self.return_sub = self.create_subscription(Int32, '/sign4return', self.main, 10)
        self.control_pub = self.create_publisher(Int32, '/control', 10)
        self.control_sub = self.create_subscription(Int32, '/update_status', self.receive, 10)
        # self.image_sub = self.create_subscription(Image, '/image', self.loop, 10)
        self.status_pub = self.create_publisher(Sign, '/sign_switch', 10)
        # self.twist = Twist()
        # self.status = 0
        self.timer1 = self.create_timer(1, self.loop) 
        self.timer2 = self.create_timer(0.5, self.status_update) 
        
        self.status = 0

        self.timer1_has = False

        

    def receive(self, msg):
        data = msg.data

        if data == 2:
            self.get_logger().info("已到达二维码区域")
            self.status = 2
        elif data == 3:
            self.get_logger().info("顺时针")
            self.status = 3
        elif data == 4:
            self.get_logger().info("逆时针")
            self.status = 4
        
        elif data == 7:
            self.get_logger().info("测试结束")
            self.status = 7
            

    def status_update(self):
        self.status_pub.publish(Sign(sign_data=self.status))


    def loop(self):
        if not self.timer1_has:
            self.control_pub.publish(Int32(data=0))
            self.status = 1
            self.timer1_has = True
        # if command == "start":
        #     self.control_pub.publish(Int32(data=0))
        #     self.status = 1
        # elif command == "stop":
        #     self.control_pub.publish(Int32(data=1))
        #     self.status = 7
        # elif command == "restart":
        #     self.control_sub.publish(Int32(data=2))
        

        

    def main(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        if msg.data == 6:
            self.get_logger().info("C区结束遥测")
            self.control_pub.publish(Int32(data=0))
            self.status = 6
            # self.status_pub.publish(Sign(sign_data=3))
        elif msg.data == 5:
            self.get_logger().info("C区遥测")
            self.control_pub.publish(Int32(data=1))
            self.status = 5
            # self.status_pub.publish(Sign(sign_data=5))
        # elif msg.data == -1:
        #     self.get_logger().info("测试测试结束了!!!!")
        #     self.status = 7
        #     # self.status_pub.publish(Sign(sign_data=7))
        # elif msg.data == 0:
        #     self.get_logger().info("开始前往二维码区域!!!!")
        #     self.status = 1
        #     # self.status_pub.publish(Sign(sign_data=1))



def main(args=None):
    try:
        rclpy.init(args=args)
        
        controller = Control()
        rclpy.spin(controller)
        
        controller.destroy_node()
        rclpy.shutdown()
    finally:
        controller.control_pub.publish(Int32(data=1))
        controller.status_pub.publish(Sign(sign_data=7))

if __name__ == '__main__':
    main()