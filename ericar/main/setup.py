from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'main'

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
    description='Main state machine and control node for ERICAR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = main.main_node:main',
        ],
    },
)
