from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ejercicio2_pkg',
            executable='move_panda_ik',
            name='move_panda_ik_node',
            output='screen'
        )
    ])
