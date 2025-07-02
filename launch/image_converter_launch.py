# launch/image_converter_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_image_encoder',  # Update if your package name is different
            executable='convert_image_encoding',  # Python filename without .py
            name='img_encoder',
            output='screen',
            parameters=[{
                'frame_rate': 30.0,
                'topic_list': ['/image_raw']
            }]
        )
    ])
