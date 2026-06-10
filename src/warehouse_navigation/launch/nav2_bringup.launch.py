"""Launch Nav2 for a single robot (used for testing)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav    = FindPackageShare("warehouse_navigation")
    pkg_nav2   = get_package_share_directory("nav2_bringup")

    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="robot_1")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([pkg_nav, "maps", "warehouse_map.yaml"]),
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_nav, "config", "nav2_params.yaml"]),
    )

    robot_name   = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file     = LaunchConfiguration("map")
    params_file  = LaunchConfiguration("params_file")

    nav2_group = GroupAction([
        PushRosNamespace(robot_name),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "namespace":       robot_name,
                "use_namespace":   "true",
                "slam":            "false",
                "map":             map_file,
                "use_sim_time":    use_sim_time,
                "params_file":     params_file,
                "autostart":       "true",
                "use_composition": "false",
            }.items(),
        ),
    ])

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        map_arg,
        params_arg,
        nav2_group,
    ])
