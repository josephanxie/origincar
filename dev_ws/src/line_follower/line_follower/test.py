#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # ROS2 Python接口库
from rclpy.node import Node  # ROS2 节点类
from geometry_msgs.msg import Twist  # 字符串消息类型
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from hobot_dnn import pyeasy_dnn as dnn
from pyzbar import pyzbar
import time
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import PointStamped


class ResNetControlNode(Node):
    def __init__(self):
        super().__init__('line_follower_nn')  # ROS2节点父类初始化
        self.get_logger().info("AON-Lab start line follower")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.image_model_pub = self.create_publisher(Image, '/image_model', 10)

        self.follow_sub = self.create_subscription(PerceptionTargets, '/racing_track_center_detection', self.follow_callback1, 10)
        
        self.follow_sub = self.create_subscription(PointStamped, '/racing_track_center_detection', self.follow_callback2, 10)

        self.twist = Twist()

        self.data = None



    def image_callback(self, msg):  # 图像订阅回调函数
        if self.data is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        cv2.circle(image, self.data, 5, (0, 0, 255), -1)
        image_msg_with_point = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.image_model_pub.publish(image_msg_with_point) 



    def follow_callback1(self, msg):
        self.get_logger().info(f"1:")
        self.get_logger().info(f"1: msg.point {msg.point}, x={msg.point.x}")
        self.data = (int(msg.point.x), int(msg.point.y + 480 - 224))
        
    def follow_callback2(self, msg):
        self.get_logger().info(f"2:123456789034")
        self.get_logger().info(f"2:msg.point {msg.point}, x={msg.point.x}")
        self.data = (int(msg.point.x), int(msg.point.y + 480 - 224))


def main(args=None):  # ROS2节点主入口main函数
    try:
        rclpy.init(args=args)  # ROS2 Python接口初始化
        node = ResNetControlNode()  # 创建ROS2节点对象并进行初始化
        rclpy.spin(node)  # 循环等待ROS2退出
        node.destroy_node()  # 销毁节点对象
        rclpy.shutdown()  # 关闭ROS2 Python接口
    finally:
        node.stop()

if __name__ == "__main__":
    main()
