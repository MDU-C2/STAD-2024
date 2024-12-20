from setuptools import setup

package_name = 'yolo_image_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='A ROS2 image subscriber node with YOLOv8 analysis',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'sub_node = yolo_image_subscriber.sub_node:main',  # Huvudfunktion för Python-noden
        ],
    },
)

