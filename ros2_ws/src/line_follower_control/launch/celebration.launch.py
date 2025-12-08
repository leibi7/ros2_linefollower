import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription(
        [
            Node(
                package="line_follower_control",
                executable="celebration_node",
                name="celebration_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

