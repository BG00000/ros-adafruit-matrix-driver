from setuptools import setup

setup(
    name='adafruit_matrix_driver',
    version='0.0.0',
    packages=['adafruit_matrix_driver'],
    # py_modules=['adafruit_matrix_driver'],
    install_requires=['setuptools'],
    author='Aiden Baker-Gabb',
    author_email='Aiden@todo.com',
    keywords=['ROS'],
    classifiers=[],
    description='Driver for Adafruit LED matrices',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'adafruit_matrix_driver_node = adafruit_matrix_driver.adafruit_matrix_driver_node:main',
            'image_publisher_node = adafruit_matrix_driver.image_publisher_node:main'
        ],
    },
)
