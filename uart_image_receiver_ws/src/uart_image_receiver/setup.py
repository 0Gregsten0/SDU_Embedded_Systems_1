from setuptools import setup
from glob import glob
import os

package_name = 'uart_image_receiver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Receive JPEG frames over UART and save the last N frames',
    license='MIT',
    entry_points={
        'console_scripts': [
            'uart_image_receiver_node = uart_image_receiver.uart_image_receiver_node:main',
            'file_transfer_node = uart_image_receiver.file_transfer_node:main',
            #'uart_image_sender_node = uart_image_receiver.uart_image_sender_node:main',
        ],
    },
)
