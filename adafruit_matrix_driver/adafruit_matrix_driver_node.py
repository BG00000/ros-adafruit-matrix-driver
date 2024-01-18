# image_subscriber_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
from rgbmatrix import RGBMatrix, RGBMatrixOptions
from PIL import Image

class AdafruitMatrixDriverNode(Node):
    def __init__(self):
        super().__init__('adafruit_matrix_driver_node')
        self.subscription = self.create_subscription(
            ImageMsg,
            'image',
            self.image_callback,
            10  # QoS profile
        )
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()

        options = RGBMatrixOptions()
        options.rows = 32
        options.chain_length = 1
        options.parallel = 1
        options.hardware_mapping = 'adafruit-hat'

        self.matrix = RGBMatrix(options = options)

    def image_callback(self, msg: ImageMsg):
        # Convert the ROS Image message to a numpy array
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pil_image = Image.fromarray(cv_image)

        # Render the image on the matrix display (replace this with your actual rendering logic)
        self.render_image(pil_image)


    def render_image(self, image):
        self.get_logger().debug("Render")
        self.matrix.SetImage(image)

def main():
    rclpy.init()
    node = AdafruitMatrixDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
