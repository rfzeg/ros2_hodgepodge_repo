from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='go_to_pose_controller',
            executable='go_to_pose_controller',
            output='screen'),
    ])
