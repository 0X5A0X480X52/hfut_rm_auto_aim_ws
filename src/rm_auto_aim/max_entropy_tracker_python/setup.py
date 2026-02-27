from setuptools import setup, find_packages

package_name = 'max_entropy_tracker_python'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tracker_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/max_entropy_tracker.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='your@email.com',
    description='Max Entropy UKF based robot pose estimator (Python implementation)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'max_entropy_tracker_node = max_entropy_tracker.max_entropy_tracker_node:main',
        ],
    },
)
