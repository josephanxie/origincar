import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/image_fox',  # 更改为你的原始图像话题
            self.image_callback,
            1)
        self.publisher = self.create_publisher(
            Image,
            '/image',  # 更改为你希望发布的目标图像话题
            1)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # self.get_logger().info("123123")
            # 将ROS Image消image_raw息转换为OpenCV格式
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # # 调整图像大小为64x48像素
            # resized_image = cv2.resize(cv_image, (640, 480))
            
            # # 将OpenCV格式的图像转换回ROS Image消息
            # resized_msg = self.cv_bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            
            # # 发布调整大小后的图像到新话题
            # self.publisher2.publish(resized_msg)
            

            # 调整图像大小为64x48像素
            resized_image = cv2.resize(cv_image, (128, 96))
            
            # 将OpenCV格式的图像转换回ROS Image消息
            resized_msg = self.cv_bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            
            # 发布调整大小后的图像到新话题
            self.publisher.publish(resized_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()