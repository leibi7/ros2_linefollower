import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x = LaunchConfiguration("x", default="0.0")
    y = LaunchConfiguration("y", default="0.0")
    z = LaunchConfiguration("z", default="0.05")

    robot_desc_path = os.path.join(
        get_package_share_directory("line_follower_robot"),
        "urdf",
        "line_follower_bot.urdf.xacro",
    )
    robot_desc = xacro.process_file(robot_desc_path).toxml()

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.05"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-entity",
                            "line_follower_bot",
                            "-topic",
                            "robot_description",
                            "-timeout",
                            "120.0",
                            "-x",
                            x,
                            "-y",
                            y,
                            "-z",
                            z,
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
