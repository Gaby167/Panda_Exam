from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit2_tutorials_dir = get_package_share_directory('moveit2_tutorials')

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit2_tutorials_dir, 'launch', 'demo.launch.py')
        ),
        launch_arguments={'rviz_config': 'panda_moveit_config_demo_empty.rviz'}.items()
    )

    move_node = Node(
        package='ejercicio1_pkg',
        executable='move_panda_node',
        name='move_panda_node',
        output='screen'
    )

    return LaunchDescription([demo_launch, move_node])
