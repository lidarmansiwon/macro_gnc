import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    odom_param_file = LaunchConfiguration(
        'odom_param_file',
        default=os.path.join(
            get_package_share_directory('navigation'),
            'param',
            'navigation_params.yaml'
        )
    )

    odom_navigation_node = Node(
        namespace='',
        package='navigation',
        executable='odom_navigation',
        name='odom_navigation',
        output='screen',
        parameters=[odom_param_file]
    )

    return LaunchDescription([
        odom_navigation_node
    ])
