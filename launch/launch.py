from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adafruit_matrix_driver',
            executable='adafruit_matrix_driver_node',
        ),
        Node(
            package='adafruit_matrix_driver',
            executable='image_publisher_node',
        )
    ])