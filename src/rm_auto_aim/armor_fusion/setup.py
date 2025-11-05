from setuptools import setup

package_name = 'armor_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Multi-camera armor detection fusion',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_camera_fusion_node = armor_fusion.multi_camera_fusion_node:main',
            'test_publisher = armor_fusion.test_publisher:main',
        ],
    },
)
