from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video2',
                'image_size': [320, 240],
                'time_per_frame': [1, 20],  # ~20 FPS
            }],
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='serial_image_sender',
            name='serial_image_sender',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 921600,
                'topic': '/image_raw',
                'frame_skip': 0,
                'jpeg_quality': 90,
                'chunk_size': 4096
            }],
            output='screen'
        )
    ])
