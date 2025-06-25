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

class ResNetControlNode(Node):
    def __init__(self):
        super().__init__('line_follower_nn')  # ROS2节点父类初始化
        self.get_logger().info("AON-Lab start line follower")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.pub = self.create_publisher(Image, '/camera/processed_image', 10)
        #self.timer = self.create_timer(0.066, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.twist = Twist()

        # 导入ResNet18模型（绝对路径）
        self.models = dnn.load('/root/dev_ws/src/line_follower/line_follower/DataSet3_train1.bin')
        
        # PID相关参数
        self.Kp = 2.0
        self.Kd = 3.0
        self.Target_value = 100.0  # 设定值初始化
        self.last_Err = 0.0  # 上一个误差值初始化
        self.total_Err = 0.0  # 误差累加初始化
        self.output = 0.0  # PID输出初始化

    def timer_callback(self):  # 定时器回调函数
        pass  # 定时器的回调函数为空，因为主要逻辑在image_callback中处理

    def image_callback(self, msg):  # 图像订阅回调函数
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 将图像调整为模型需要的输入尺寸
        image_resized = cv2.resize(image, (224, 224))

        # 确保图像为uint8类型并转换为NV12格式
        if image_resized.dtype != np.uint8:
            image_resized = image_resized.astype(np.uint8)

        # 转换为NV12格式
        yuv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2YUV_I420)
        
        # 模型推理
        outputs = self.models[0].forward(np.array(yuv_image, dtype=np.uint8).flatten())
        outputs = outputs[0].buffer
        x, y = int(224 * outputs[0][0][0]), int(224 * outputs[0][1][0])

        # PID角速度控制目标计算
        self.Error = self.Target_value - 1.0 * x  # 计算偏差
        self.total_Err = self.total_Err + self.Error  # 偏差累加
        self.output = self.Kp * self.Error + self.Kd * (self.Error - self.last_Err)  # PID运算
        self.last_Err = self.Error  # 将本次偏差赋给上次偏差

        self.get_logger().info(f"( {x:03d}, {y:03d} )  output = {int(self.output):03d}")
        self.twist.linear.x = 0.25
        self.twist.angular.z = self.output / 114.0 + 0.22
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):  # ROS2节点主入口main函数
    rclpy.init(args=args)  # ROS2 Python接口初始化
    node = ResNetControlNode()  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)  # 循环等待ROS2退出
    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS2 Python接口

if __name__ == "__main__":
    main()
