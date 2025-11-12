from setuptools import setup
import os

package_name = 'image_raw_recoder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['opencv-python', 'numpy'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='ROS2 recorder package: write MP4 from /image_raw or convert ros2 bag to MP4',
    entry_points={
        'console_scripts': [
            'recorder_node = image_raw_recoder.recorder_node:main',
            'bag_to_mp4 = image_raw_recoder.bag_to_mp4:main',
            'multi_recorder = image_raw_recoder.multi_recorder:main',
        ],
    },
)
