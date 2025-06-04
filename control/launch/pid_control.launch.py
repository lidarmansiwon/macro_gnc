import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_file = LaunchConfiguration(
        'param_file',
        default=os.path.join(
            get_package_share_directory('control'),
            'param',
            'control_params.yaml'
        )
    )

    pid_control_node = Node(
        package='control',
        executable='pid_control',
        name='PID_control',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        pid_control_node
    ])
