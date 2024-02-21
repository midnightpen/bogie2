from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_py_pkg',
            executable='py_pub_node',
            name='simple_pub',
            output='screen'
        ),
        Node(
            package='my_py_pkg',
            executable='py_sub_node',
            name='simple_sub',
        ),
    ])