from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory("line_follower_world"), "worlds", "line_world.world"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    verbose = LaunchConfiguration("verbose", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="true",
                description="Enable verbose Gazebo output",
            ),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.dirname(world_path)),
            SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", os.path.dirname(world_path)),
            SetEnvironmentVariable("GAZEBO_SYSTEM_PLUGIN_PATH", "/opt/ros/humble/lib"),
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    world_path,
                ],
                output="screen",
                additional_env={
                    "GAZEBO_SYSTEM_PLUGIN_PATH": "/opt/ros/humble/lib",
                    "GAZEBO_PLUGIN_PATH": "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
                    "GAZEBO_MODEL_PATH": f"{os.path.dirname(world_path)}:/usr/share/gazebo-11/models",
                    "GAZEBO_RESOURCE_PATH": f"{os.path.dirname(world_path)}:/usr/share/gazebo-11",
                },
            ),
        ]
    )
