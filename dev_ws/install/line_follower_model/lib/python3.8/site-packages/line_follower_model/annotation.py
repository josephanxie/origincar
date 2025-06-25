import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2 as cv
import uuid
import os
import numpy as np

from sensor_msgs.msg import Image

class ImageSubscriber(Node):
    
    def __init__(self):
        super().__init__('annotation')  # 创建节点并命名为 'ImageSubscriber'
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            1
        )
        # 创建 CvBridge 实例
        self.bridge = CvBridge()
        self.x = -1
        self.y = -1
        self.uuid = -1
        self.image = np.zeros((640, 224, 3), dtype=np.uint8)
        self.initialize = False
        
        if not os.path.exists('./image_dataset'):
            os.makedirs('./image_dataset')
            
        # 设置 OpenCV 窗口属性
        cv.namedWindow("capture image", cv.WINDOW_NORMAL)
        cv.resizeWindow("capture image", 640, 224)

        self.subscription

    def mouse_callback(self, event, x, y, flags, userdata):
        if event == cv.EVENT_LBUTTONDOWN:
            imageWithCircle = userdata.copy()
            self.x = x
            self.y = y
            cv.circle(imageWithCircle, (x, y), 5, (0, 0, 255), -1)
            cv.imshow("capture image", imageWithCircle)

    # 订阅回调函数
    def listener_callback(self, msg):
        keyValue = cv.waitKey(1)
        # 检测到按下回车键，则获取一张新的图像
        if keyValue == 13:
            captureImage = self.bridge.imgmsg_to_cv2(msg)
            cropImage = captureImage[128:352, :, :].copy()
            if not self.initialize:
                self.image = cropImage.copy()
                self.initialize = True
            # 注册鼠标回调
            cv.setMouseCallback("capture image", self.mouse_callback, cropImage)
            cv.imshow("capture image", cropImage)
            if self.x != -1 and self.y != -1:
                self.uuid = 'xy_%03d_%03d_%s' % (self.x, self.y, uuid.uuid1())  # 保存上一次标定的结果
                cv.imwrite('./image_dataset/' + self.uuid + '.jpg', self.image)  # 载入新的图像
                self.image = cropImage.copy()
                self.x = -1
                self.y = -1

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
