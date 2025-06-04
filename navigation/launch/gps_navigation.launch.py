import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_file = LaunchConfiguration(
        'param_file',
        default=os.path.join(
            get_package_share_directory('navigation'),
            'param',
            'navigation_params.yaml'
        )
    )

    gps_navigation_node = Node(
        package='navigation',
        executable='gps_navigation',
        name='gps_navigation',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        gps_navigation_node
    ])
