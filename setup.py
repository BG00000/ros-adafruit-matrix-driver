from setuptools import setup

package_name = "adafruit_matrix_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aiden Baker-Gabb",
    maintainer_email="aidenbakergabb@gmail.com",
    description="A ROS2 driver for the Adafruit 32x32 RGB LED Matrix",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'adafruit_matrix_driver_node = adafruit_matrix_driver.adafruit_matrix_driver_node:main',
            'image_publisher_node = adafruit_matrix_driver.image_publisher_node:main'
        ],
    },
)
