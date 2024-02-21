# sudo apt install ros-iron-slam-toolbox
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory('bogie_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_dir = LaunchConfiguration(
        'slam_params_file',
        default = os.path.join(share_dir,'config', 'slam_toolbox_param.yaml')
    )
    rviz_config_file = os.path.join(share_dir,'config', 'map.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
             default_value = 'false',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value = slam_params_dir,
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
        ),
        Node(
            parameters=[
                slam_params_dir,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])