from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_processor',
            executable='image_publisher',
            name='image_publisher',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='image_processor',
            executable='image_listener',
            name='image_listener',
            output='screen',
            emulate_tty=True
        )
    ])