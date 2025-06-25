#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # ROS2 Python接口库
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node  # ROS2 节点类
from geometry_msgs.msg import Twist, PointStamped 
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from hobot_dnn import pyeasy_dnn as dnn
from pyzbar import pyzbar
import time
from std_msgs.msg import Int32

from postprocess import postprocess, draw_bboxs
from ai_msgs.msg import PerceptionTargets 
import yolov5_post

def print_properties(pro):
    print("tensor type:", pro.tensor_type)
    print("data type:", pro.dtype)
    print("layout:", pro.layout)
    print("shape:", pro.shape)

# 依据模型input_type_rt决定是否需要进行数据格式转换（本实例所用模型为nv12输入）
# https://developer.horizon.ai/forumDetail/98129467158916308
def bgr2nv12_opencv(image):
    height, width = image.shape[0], image.shape[1]
    area = height * width
    yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
    y = yuv420p[:area]
    uv_planar = yuv420p[area:].reshape((2, area // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))

    nv12 = np.zeros_like(yuv420p)
    nv12[:height * width] = y
    nv12[height * width:] = uv_packed
    return nv12

def convert_bgr_to_nv12(cv_image):
    yuv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
    y_channel = yuv_image[:, :, 0]
    u_channel = yuv_image[::2, ::2, 1]
    v_channel = yuv_image[::2, ::2, 2]
    uv_channel = np.empty((u_channel.shape[0], u_channel.shape[1] * 2), dtype=u_channel.dtype)
    uv_channel[:, ::2] = u_channel
    uv_channel[:, 1::2] = v_channel
    nv12_image = np.concatenate((y_channel.flatten(), uv_channel.flatten()))
    return nv12_image

class ResNetControlNode(Node):
    def __init__(self):
        super().__init__('line_follower_nn')  # ROS2节点父类初始化
        self.get_logger().info("AON-Lab start line follower")
        self.bridge = CvBridge()
        self.follow_group = MutuallyExclusiveCallbackGroup()
        self.follow_sub = self.create_subscription(Image, '/image_fox', self.follow_line, 10, callback_group=self.follow_group)
        

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.image_model_pub = self.create_publisher(Image, '/image_model', 10)


        self.control_group = MutuallyExclusiveCallbackGroup()
        self.control_pub = self.create_publisher(Int32, '/update_status', 10, callback_group=self.control_group)
        self.control_sub = self.create_subscription(Int32, '/control', self.receive, 1, callback_group=self.control_group)

        self.twist = Twist()

        self.models = [dnn.load('/root/follow723_2.bin')]

        
        # PID相关参数
        self.Kp = 3.0
        self.Kd = 2.0
        self.last_Err = 0.0  # 上一个误差值初始化
        self.total_Err = 0.0  # 误差累加初始化
        self.output = 0.0  # PID输出初始化

        self.run = False

        self.qr_result = 0

        self.point = (0, 0)

        self.flag = False
        self.ob_run = True
        self.qrcode_group = MutuallyExclusiveCallbackGroup()
        self.qrcode_sub = self.create_subscription(Image, '/image_fox', self.qrcode, 10, callback_group=self.qrcode_group)

        self.ob_group = MutuallyExclusiveCallbackGroup()
        self.ob_sub = self.create_subscription(PerceptionTargets, 'hobot_dnn_detection', self.avoid_obstraction, 10, callback_group=self.ob_group)


    def receive(self, msg):
        if msg.data == 0:
            self.run = True
            self.get_logger().info("进入开始状态")
        elif msg.data == 1:
            self.run = False
            self.get_logger().info("处于停止状态")
        


    def qrcode(self, msg):
        if not self.run:
            return
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

        self.run = False
        time.sleep(0.1)
        self.stop()

        self.control_pub.publish(Int32(data=2))  # 到达二维码区域
        time.sleep(0.5)
        self.get_logger().info(f"qr_result:{self.qr_result} ")
        self.control_pub.publish(Int32(data=self.qr_result))  # 二维码内容

        return True

    def follow_line(self, msg):
        if not self.run:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cropped_image = img[160:384,:,:]
        image_resized = cv2.resize(cropped_image, (224, 224), interpolation=cv2.INTER_AREA)
        
        nv12_data = convert_bgr_to_nv12(image_resized)

        outputs = self.models[0][0].forward(np.frombuffer(nv12_data, dtype=np.uint8))
        outputs = outputs[0].buffer
        
        x = int(outputs[0][0][0] * 640)
        y = int(outputs[0][1][0] * 224)
        # self.get_logger().info(f"线中心点为 x={x}, y={y}")
        cv2.circle(cropped_image, (x, y), 5, (255, 0, 0), -1)
        image_msg_with_point = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
        self.image_model_pub.publish(image_msg_with_point) 
        # return

        self.twist.linear.x = 0.5
        # return
        self.Error = 100 - 1.0 * x / 640 * 224  # 计算偏差
        self.total_Err = self.total_Err + self.Error  # 偏差累加
        self.output = self.Kp * self.Error + self.Kd * (self.Error - self.last_Err)  # PID运算
        self.last_Err = self.Error  # 将本次偏差赋给上次偏差

        self.twist.angular.z = self.output / 414.0

        self.get_logger().info(f"线中心点为 x={x}, y={y}, 角速度 = {self.twist.angular.z}")
        self.cmd_vel_pub.publish(self.twist)

    def avoid_obstraction(self, msg):
        if not self.ob_run:
            return
        print("\n \033[31m---\033[0m This Frame: FPS = %d  \033[31m---\033[0m"%msg.fps)

        max_target = [0, 0, 0]
        x, y, S = 0, 0, 0
        has_flag, flag = False, False
        for num, target in enumerate(msg.targets):
            if target.rois[0].confidence > 0.6:
                # print("Type: %s, x_offset=%d, y_offset=%d, height=%d, width=%d, conf=%.2f"%(target.rois[0].type, 
                # target.rois[0].rect.x_offset,
                # target.rois[0].rect.y_offset,
                # target.rois[0].rect.height,
                # target.rois[0].rect.width,
                # target.rois[0].confidence))
                x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2
                y = target.rois[0].rect.y_offset + target.rois[0].rect.height 
                S = target.rois[0].rect.height * target.rois[0].rect.width #TODO
                
                
                self.get_logger().info(f"物体位置({x}, {y}), 面积为{S}")
                # return
                if y > 320:
                    self.stop()
                    self.flag = True
                    temp = x - 320
                    if -10 < temp <= 0:
                        temp = -10
                    elif 0 <= temp < 10:
                        temp = 10
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = 1 * 800 / temp
                    self.get_logger().info(f"避障中...角速度 = {self.twist.angular.z}")
                    self.cmd_vel_pub.publish(self.twist)
                    self.flag = 1 if temp < 0 else 2 
                    has_flag = True

        if not has_flag and  self.flag : #已远离障碍物
            self.ob_run = False
            self.stop()
            if self.flag == 1:
                self.twist.linear.x = 0.3
                self.twist.angular.z = 2.2
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(1.5)
            # else:
            #     self.twist.linear.x = 0.3
            #     self.twist.angular.z = -2.2
            #     self.cmd_vel_pub.publish(self.twist)
            #     time.sleep(1.5)
            self.run = True
            self.ob_run = True
            self.flag = False 

                        

                # self.run = True


    
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
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(node)
        executor.spin()
        node.destroy_node()  # 销毁节点对象
    finally:
        executor.shutdown()
        rclpy.shutdown() 
        node.stop()

if __name__ == "__main__":
    main()
