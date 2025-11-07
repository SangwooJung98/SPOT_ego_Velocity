from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_msgs',
            executable='spot_msgs_node',
            name='spot_msgs_node',
            output='screen'
        )
    ])