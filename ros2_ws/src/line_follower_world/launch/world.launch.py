from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_pkg_share = get_package_share_directory("line_follower_world")
    world_path = os.path.join(world_pkg_share, "worlds", "line_world.world")
    models_path = os.path.join(world_pkg_share, "models")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    verbose = LaunchConfiguration("verbose", default="true")
    model_path_env = f"{models_path}:/usr/share/gazebo-11/models"
    resource_path_env = (
        f"{os.path.dirname(world_path)}:{models_path}:/usr/share/gazebo-11"
    )

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
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path_env),
            SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", resource_path_env),
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
                    "GAZEBO_MODEL_PATH": model_path_env,
                    "GAZEBO_RESOURCE_PATH": resource_path_env,
                },
            ),
        ]
    )
