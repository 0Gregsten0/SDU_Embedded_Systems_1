from setuptools import setup
from glob import glob
import os


package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gregsten',
    maintainer_email='gt_schweiz@gmx.ch',
    description='Minimal rclpy package for ROS 2 Jazzy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_image_sender = my_robot_package.serial_image_sender:main',
            # 'image_read = my_robot_package.image_read:main',
            # 'image_edit = my_robot_package.image_edit:main',
            # 'image_save = my_robot_package.image_save:main',
            # 'talker = my_robot_package.talker:main',
            # 'image_logger = my_robot_package.image_logger:main',
        ],
    },
)
