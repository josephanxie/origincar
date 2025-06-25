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
from std_msgs.msg import Int32

from postprocess import postprocess


def color_block_finder(img, lowerb, upperb, min_w=0, min_h=0):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_bin = cv2.inRange(img_hsv, lowerb, upperb)
    
    contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_h, max_w = img.shape[0], img.shape[1]
    max_rect = None

    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)

        if w >= min_w and w <= max_w and h >= min_h and h <= max_h:
            if max_rect is None:
                max_rect = (x, y, w, h)
            else:
                if w * h > max_rect[2] * max_rect[3]:
                    max_rect = (x, y, w, h)
    return max_rect

class ResNetControlNode(Node):
    def __init__(self):
        super().__init__('line_follower_nn')  # ROS2节点父类初始化
        self.get_logger().info("AON-Lab start line follower")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_model_pub = self.create_publisher(Image, '/image_model', 10)


        self.control_pub = self.create_publisher(Int32, '/update_status', 10)
        self.control_sub = self.create_subscription(Int32, '/control', self.receive, 10)

        self.twist = Twist()

        # 导入ResNet18模型（绝对路径）
        self.models = [dnn.load('/root/follow2.bin'), dnn.load('/root/ob2.bin')]
        
        # PID相关参数
        self.Kp = 2.0
        self.Kd = 3.0
        self.Target_value = 320.0  # 设定值初始化
        self.last_Err = 0.0  # 上一个误差值初始化
        self.total_Err = 0.0  # 误差累加初始化
        self.output = 0.0  # PID输出初始化

        self.has_check_qr = 0
        self.run = False

        self.qr_result = 0

        self.Kv = 0.5

    def receive(self, msg):
        if msg.data == 0:
            self.run = True
            self.get_logger().info("进入开始状态")
        elif msg.data == 1:
            self.run = False
            self.get_logger().info("处于停止状态")
        

    def check_is_stop(self, img):
        x, y, w, h = color_block_finder(img, (0, 0, 150) , (50, 50, 200))
        self.get_logger().info(f"色块含量为: {w * h / (224*224)}")

    def qrcode(self, img):
        result = pyzbar.decode(img)
        if len(result) == 0:
            if self.has_check_qr > 0:
                self.has_check_qr += 1  
                if self.has_check_qr > 4: # 到二维码区了
                    self.run = False

                    time.sleep(0.1)

                    self.twist.linear.x = 0.05
                    self.twist.angular.z = -5.0

                    time.sleep(0.2)

                    self.stop()

                    self.control_pub.publish(Int32(data=2))  # 到达二维码区域
                    time.sleep(0.5)
                    self.get_logger().info(f"qr_result:{self.qr_result} ")
                    self.control_pub.publish(Int32(data=self.qr_result))  # 二维码内容
                    return False, False
            return None, None
            
        
        if self.has_check_qr == 0:
            self.has_check_qr = 1
            self.qr_result = result[0].data.decode()
            if self.qr_result == "AntiClockWise":
                self.qr_result = 4
            elif self.qr_result == "ClockWise":
                self.qr_result = 3
            else:
                raise ValueError("警告警告, 二维码内容不对")
        self.get_logger().info(f"{result}")

        p1, p2 = result[0].polygon[0], result[0].polygon[2]
        return (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2


    def image_callback(self, msg):  # 图像订阅回调函数
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # image = cv2.imread("046be.jpg")

        image_resized = cv2.resize(image, (672, 672), interpolation=cv2.INTER_AREA)
        yuv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2YUV_I420)
        
        outputs = self.models[1][0].forward(yuv_image)
        prediction_bbox = postprocess(outputs, model_hw_shape=(672, 672), origin_image=image_resized)
        self.get_logger().info(f"prediction_bbox={prediction_bbox}")
        image_msg_with_point = self.bridge.cv2_to_imgmsg(image_resized, "bgr8")
        self.image_model_pub.publish(image_msg_with_point) 
        return
        # exit()
        # outputs = outputs[0].buffer
        
        self.get_logger().info(f"outputs={outputs[0].shape}")


        roi = {
            'left': 0,
            'top': 480 - 224,
            'right': 640 - 1,
            'bottom': 480 - 1
        }
        #cropped_image = image[roi['top']:roi['bottom'] + 1, roi['left']:roi['right'] + 1]
        cropped_image = image[160:384,:,:]
        #self.get_logger().info(f"cropped_image.shape={cropped_image.shape}")
        # 将图像调整为模型需要的输入尺寸
        image_resized = cv2.resize(cropped_image, (224, 224))

        # 确保图像为uint8类型并转换为NV12格式
        if image_resized.dtype != np.uint8:
            image_resized = image_resized.astype(np.uint8)

        # 转换为NV12格式
        yuv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2YUV_I420)
        
        outputs = self.models[1][0].forward(yuv_image)
        self.get_logger().info(f"outputs_length={outputs}")
        outputs = outputs[0].buffer
        
        self.get_logger().info(f"outputs={outputs[0].shape}")
        exit()



        # 模型推理
        outputs = self.models[0][0].forward(yuv_image)
        
        outputs = outputs[0].buffer
        
        x = int((outputs[0][0][0] * 112 + 112) * 900 / 224)
        y = int((outputs[0][1][0] * 112 + 112))
        #x = int((outputs[0][0][0])*640)
        #y = int((outputs[0][1][0])*224)
        self.get_logger().info(f"x={outputs[0][0][0]}, y={outputs[0][1][0]}")
        #return
        cv2.circle(cropped_image, (x, y), 5, (0, 0, 255), -1)
        image_msg_with_point = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
        self.image_model_pub.publish(image_msg_with_point) 
        #return
    

        temp = x - 640/2
        if -20 < x < 0:
            temp = -20
        elif 0 < x < 20:
            temp = 20
        
        self.twist.linear.x = self.Kv
        self.twist.angular.z = -(x-320) / 300

        self.get_logger().info(f"( {x}, {y} )  角速度 = {self.twist.angular.z}")
        self.cmd_vel_pub.publish(self.twist)

    
    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info("Stop")
        self.run = False

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
