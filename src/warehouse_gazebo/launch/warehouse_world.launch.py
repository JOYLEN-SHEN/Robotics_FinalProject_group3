"""Launch the warehouse Gazebo world (without robots)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_description = FindPackageShare("warehouse_description")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_description, "worlds", "warehouse.world"]),
        description="Full path to the Gazebo world file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (Gazebo) clock",
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Set to false to run headless",
    )
    verbose_arg = DeclareLaunchArgument(
        "verbose", default_value="false",
        description="Set to true for verbose Gazebo output",
    )

    world = LaunchConfiguration("world")
    gui   = LaunchConfiguration("gui")

    # Set Gazebo model path so it can find our custom models
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            PathJoinSubstitution([pkg_description, "meshes"]),
            ":",
            "/usr/share/gazebo-11/models",
        ],
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world,
            "verbose": LaunchConfiguration("verbose"),
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        launch_arguments={"gui": gui}.items(),
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gui_arg,
        verbose_arg,
        gazebo_model_path,
        gazebo_server,
        gazebo_client,
    ])
