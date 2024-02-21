# sudo apt install ros-iron-cartographer
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
import os


def generate_launch_description():
    share_dir = get_package_share_directory('bogie_navigation')
    rviz_config_file = os.path.join(share_dir, 'config', 'map.rviz')

    return LaunchDescription([
        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='cartographer_ros', executable='cartographer_node', output='screen',
            arguments=[
            	'-configuration_directory', get_package_share_directory('bogie_navigation')+'/config',
            	'-configuration_basename', 'cartographer.lua'
               
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.02', '-publish_period_sec', '1.0']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
