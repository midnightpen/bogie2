#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros'),'launch','rplidar_a1_launch.py']
    )

    Camera_launch_path = PathJoinSubstitution(
        [FindPackageShare('bogie_navigation'),'launch','qr_camera.launch.py']
    )

    bogie_description_launch = PathJoinSubstitution(
        [FindPackageShare('bogie_description'),'launch','display.launch.py']
    )

    Imu_launch_path = PathJoinSubstitution(
        [FindPackageShare('serial_imu'),'launch','imu.launch.py']
    )

    Ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('bogie_navigation'),'config','ekf.yaml']
    )
    
    return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch_path)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bogie_description_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(Imu_launch_path)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(Camera_launch_path)
            ),
            Node(
                package='bogie_bringup',
                executable='base_controller',
                name='base_controller_node',
                output='screen',
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                remappings=[("/odometry/filtered","/odom")],
                parameters=[
                    Ekf_config_path
                ],
            ),
    ])