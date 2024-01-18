# image_publisher_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import math

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(Image, 'image', 10)  # QoS profile
        self.timer = self.create_timer(0.1, self.publish_image)  # Publish every 1 second
        self.cv_bridge = CvBridge()

    def publish_image(self):


        # image = np.random.rand(32,32,3) * 255
        # image[:,:,0] = 0
        # image[:,:,2] = 0
        # image_uint8 = image.astype("uint8")

        # # Convert the OpenCV image to a ROS Image message
        # ros_image_msg = self.cv_bridge.cv2_to_imgmsg(image_uint8, encoding='bgr8')

        # # Publish the ROS Image message
        # self.publisher.publish(ros_image_msg)
        # self.get_logger().debug('Published image')

        try:
            # Load your own PNG image from disk using OpenCV
            image_path = os.path.expanduser('~/Downloads/GR1.png')

            # Read the image
            original_image = cv2.imread(image_path, cv2.IMREAD_COLOR)

            if original_image is None:
                raise FileNotFoundError(f"Failed to load the image from {image_path}")

            # Resize the image to 32x32
            resized_image = cv2.resize(original_image, (32, 32))

            # Optionally, you can perform any additional image processing here
            # For example, set red and blue channels to 0
            # resized_image[:, :, 0] = 0
            # resized_image[:, :, 2] = 0

            # Calculate pulsating factor based on elapsed time
            elapsed_time = rclpy.clock.Clock().now().nanoseconds * 1e-9
            pulsating_frequency = 2.0  # Adjust the frequency of the pulsation
            pulsating_factor = 0.5 + 0.5 * math.sin(elapsed_time * pulsating_frequency)

            # Apply the pulsating factor to the image
            pulsating_image = (resized_image * pulsating_factor).astype(np.uint8)

            # Convert the OpenCV image to a ROS Image message
            ros_image_msg = self.cv_bridge.cv2_to_imgmsg(pulsating_image, encoding='bgr8')

            # Publish the ROS Image message
            self.publisher.publish(ros_image_msg)
            self.get_logger().debug('Published image')

        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')


def main():
    rclpy.init()
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
