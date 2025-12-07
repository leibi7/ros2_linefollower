import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    start_control = LaunchConfiguration("start_control", default="true")

    world_pkg = get_package_share_directory("line_follower_world")
    robot_pkg = get_package_share_directory("line_follower_robot")
    control_pkg = get_package_share_directory("line_follower_control")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(world_pkg, "launch", "world.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "x": "0.0",
            "y": "0.0",
            "z": "0.05",
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_pkg, "launch", "control.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_control),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_control", default_value="true"),
            world_launch,
            spawn_robot_launch,
            control_launch,
        ]
    )
