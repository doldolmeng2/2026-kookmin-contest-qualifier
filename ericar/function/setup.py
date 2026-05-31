from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'function'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ericar',
    maintainer_email='ktypet13@hanyang.ac.kr',
    description='Utility nodes: IMU/LiDAR/camera visualizers and manual control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_visualizer   = function.imu_visualizer:main',
            'lidar_visualizer = function.lidar_visualizer:main',
            'camera_viewer    = function.camera_viewer:main',
            'manual_control   = function.manual_control:main',
        ],
    },
)
