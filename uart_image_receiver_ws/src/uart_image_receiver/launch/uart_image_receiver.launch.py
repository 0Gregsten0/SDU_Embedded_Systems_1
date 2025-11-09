from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart_image_receiver',
            executable='uart_image_receiver_node',
            name='uart_image_receiver_node',
            parameters=[{
                'port': '/dev/ttyTHS1',
                'baud': 921600,
                'output_dir': 'uart_frames',
                'keep': 10,
                'print_every': 1,
            }],
            output='screen'
        ),
        # Node(
        #     package='uart_image_receiver',
        #     executable='uart_image_sender_node',
        #     name='uart_image_sender_node',
        #     parameters=[{
        #         'port': '/dev/ttyTHS1',
        #         'baud': 921600,
        #         'dir': 'transferred_frames',
        #         'interval': 1.0,
        #         'resend_same': False,
        #     }],
        #     output='screen'
        # ),
        Node(
            package='uart_image_receiver',
            executable='file_transfer_node',
            name='file_transfer_node',
            parameters=[{
                'src_dir': 'uart_frames',
                'dst_dir': 'transferred_frames',
                'pattern': '*.jpg',
                'interval': 0.5,
                'keep_src': False,
                'max_per_cycle': 0,
            }],
            output='screen'
        )
    ])
