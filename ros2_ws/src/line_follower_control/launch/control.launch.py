from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="line_follower_control",
                executable="line_follower_node",
                name="line_follower_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="line_follower_control",
                executable="goal_monitor_node",
                name="goal_monitor",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
