# image_publisher_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(Image, 'image', 10)  # QoS profile
        self.timer = self.create_timer(1.0, self.publish_image)  # Publish every 1 second
        self.cv_bridge = CvBridge()

    def publish_image(self):
        # try:
        # Load the greenroom logo from disk using OpenCV
        logo_path = os.path.join(os.path.dirname(__file__), 'resource', 'GR1.png')
        cv_image = cv2.imread(logo_path, cv2.IMREAD_COLOR)

        self._logger.info(f"Image type {__file__} {type(cv_image)}")

        # Convert the OpenCV image to a ROS Image message
        ros_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Publish the ROS Image message
        self.publisher.publish(ros_image_msg)
        self.get_logger().info('Published greenroom logo on topic image')

        # except Exception as e:
        #     self.get_logger().error(f'Error publishing image: {str(e)}')

def main():
    rclpy.init()
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
