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


        self.ob_sub = self.create_subscription(PerceptionTargets, 'hobot_dnn_detection', self.avoid_obstraction, 10)

        self.control_pub = self.create_publisher(Int32, '/update_status', 10)
        self.control_sub = self.create_subscription(Int32, '/control', self.receive, 10)

        self.twist = Twist()

        # 导入ResNet18模型（绝对路径）721修改了路径
        self.models = [dnn.load('/root/follow2.bin')]

        
        # PID相关参数
        self.Kp = 2.0
        self.Kd = 2.0
        self.Target_value = 320.0  # 设定值初始化
        self.last_Err = 0.0  # 上一个误差值初始化
        self.total_Err = 0.0  # 误差累加初始化
        self.output = 0.0  # PID输出初始化

        self.run = True

        self.qr_result = 0


    def receive(self, msg):
        if msg.data == 0:
            self.run = True
            self.get_logger().info("进入开始状态")
        elif msg.data == 1:
            self.run = False
            self.get_logger().info("处于停止状态")
        


    def qrcode(self, img):
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
        time.sleep(0.3)
        self.stop()

        self.control_pub.publish(Int32(data=2))  # 到达二维码区域
        time.sleep(0.5)
        self.get_logger().info(f"qr_result:{self.qr_result} ")
        self.control_pub.publish(Int32(data=self.qr_result))  # 二维码内容

        return True

    def follow_line(self, img):
        cropped_image = img[160:384,:,:]
        image_resized = cv2.resize(img, (224, 224)).astype(np.uint8)
        
        #yuv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2YUV_I420)
        # yuv_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2YUV_I420)
        nv12_data = bgr2nv12_opencv(image_resized)
        outputs = self.models[0][0].forward(nv12_data)
        
        outputs = outputs[0].buffer
        print(outputs.shape)
        x = int((outputs[0][0][0] * 112 + 112) * 900 / 224)
        y = int((outputs[0][1][0] * 112 + 112))

        #cv2.circle(cropped_image, (x, y), 5, (0, 0, 0), -1)
        image_msg_with_point = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
        self.image_model_pub.publish(image_msg_with_point) 
        # return
        self.twist.linear.x = 0.2

        self.Error = self.Target_value - 1.0 * x  # 计算偏差
        self.total_Err = self.total_Err + self.Error  # 偏差累加
        self.output = self.Kp * self.Error + self.Kd * (self.Error - self.last_Err)  # PID运算
        self.last_Err = self.Error  # 将本次偏差赋给上次偏差

        self.twist.angular.z = self.output / 414.0

        self.get_logger().info(f"线中心点为 x={x}, y={y}, 角速度 = {self.twist.angular.z}")
        self.cmd_vel_pub.publish(self.twist)

    def avoid_obstraction(self, msg):
        print("\n \033[31m---\033[0m This Frame: FPS = %d  \033[31m---\033[0m"%msg.fps)

        for num, target in enumerate(msg.targets):
            print("Traget \033[0;32;40m%d\033[0m: "%num, end="")
            if target.rois[0].confidence > 0.6:
                
            print("Type: %s, x_offset=%d, y_offset=%d, height=%d, width=%d, conf=%.2f"%(target.rois[0].type, 
            target.rois[0].rect.x_offset,
            target.rois[0].rect.y_offset,
            target.rois[0].rect.height,
            target.rois[0].rect.width,
            target.rois[0].confidence))
        return

        t = time.time()
        img_ = img.copy()
        image_resized = cv2.resize(img, (640, 640), interpolation=cv2.INTER_AREA)
        nv12_data = bgr2nv12_opencv(image_resized)
        self.get_logger().info(f"图片处理耗时: {(time.time()-t)*1000}ms")
        
        t = time.time()
        outputs = self.models[1][0].forward(nv12_data)
        self.get_logger().info(f"推理耗时: {(time.time()-t)*1000}ms")
        
        t = time.time()
        prediction_bbox = yolov5_post.postprocess(outputs, 640, 1, img_.shape[0], img_.shape[1])
        for box in prediction_bbox:
            box[0] *= 1.33
            box[1] /= 1.33
            box[2] *= 1.33
            box[3] /= 1.33
        # prediction_bbox = postprocess(outputs, (672, 672), img_)
        self.get_logger().info(f"后处理耗时: {(time.time()-t)*1000}ms")
        
        t = time.time()
        draw_bboxs(img_, prediction_bbox)
        self.get_logger().info(f"画框: {(time.time()-t)*1000}ms")
        
        self.get_logger().info(f"prediction_bbox={prediction_bbox}")
        # for box in prediction_bbox:
        #     cv2.rectangle(img_, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0,0,255),2)
        # image_msg_with_point = self.bridge.cv2_to_imgmsg(img_, "bgr8")
        # self.image_model_pub.publish(image_msg_with_point) 

        if len(prediction_bbox) == 0:
            return
        
        
        # 有障碍物
        max_object = [0, 0, 0]
        for bbox in prediction_bbox:
            coor = np.array(bbox[:4], dtype=np.int32)
            c1, c2 = (coor[0], coor[1]), (coor[2], coor[3])

            x = (c1[0] + c2[0]) // 2
            y = min(c1[1], c2[1])

            S = abs((c2[0] - c1[0]) * (c2[1] - c1[1]))
            #721注释 722
            if S > max_object[2]:
                max_object = [x, y, S]
        
        self.get_logger().info(f"最近物体位置({x}, {y}), 面积为{S}")
        return # return 结束
        # 721注释 722
        if S > 40000:
            self.stop()
            temp = max_object[0] - 320
            if -20 < temp < 0:
                temp = -20
            elif 0 < temp < 20:
                temp = 20
            
            self.twist.linear.x = 0.2
            self.twist.angular.z = 1 * 700 / temp
            self.get_logger().info(f"避障中...角速度 = {self.twist.angular.z}")
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1)
            
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.5)
            
            self.twist.linear.x = 0.2
            self.twist.angular.z = -1 * 700 / temp
            self.get_logger().info(f"避障中...角速度 = {self.twist.angular.z}")
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(1)
            self.run = False

        


    def image_callback(self, msg):  # 图像订阅回调函数
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #self.get_logger().info(f"image 图片大小为{image.shape}")
        if not self.run:
            return

        
        
        
        


        if not self.qrcode(image):
            self.follow_line(image) #722
    
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
