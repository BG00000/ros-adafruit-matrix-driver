# image_subscriber_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class AdafruitMatrixDriverNode(Node):
    def __init__(self):
        super().__init__('adafruit_matrix_driver_node')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10  # QoS profile
        )
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()

    def image_callback(self, msg: Image):
        try:
            # Convert the ROS Image message to a numpy array
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Render the image on the matrix display (replace this with your actual rendering logic)
            self.render_image(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def render_image(self, image):
        # Implement your rendering logic here
        # For example, display the image using OpenCV
        cv2.imshow('Image Display', image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AdafruitMatrixDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
