"""
Launch SLAM toolbox for a single robot to build the warehouse map.

Uses OpaqueFunction to resolve the robot_name substitution at launch time,
then builds plain Python strings for the namespaced TF frame overrides.
This avoids the pitfall of passing a list-of-substitutions as a Node parameter
(which would be interpreted as a parameter array, not a concatenated string).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_slam(context, *args, **kwargs):
    robot_name   = context.perform_substitution(LaunchConfiguration("robot_name"))
    use_sim_time = context.perform_substitution(LaunchConfiguration("use_sim_time"))
    slam_params  = context.perform_substitution(LaunchConfiguration("slam_params_file"))

    # Build namespaced frame names as plain Python strings
    base_frame = f"{robot_name}/base_link"
    odom_frame  = f"{robot_name}/odom"

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=robot_name,
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time == "true",
                "base_frame":   base_frame,
                "odom_frame":   odom_frame,
            },
        ],
        remappings=[
            ("/scan",      f"/{robot_name}/scan"),
            ("/odom",      f"/{robot_name}/odom"),
            # Use namespaced TF to match RSP/diff_drive and nav2 nodes
            ("/tf",        f"/{robot_name}/tf"),
            ("/tf_static", f"/{robot_name}/tf_static"),
        ],
        output="screen",
    )

    return [slam_node]


def generate_launch_description():
    pkg_nav = FindPackageShare("warehouse_navigation")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name", default_value="robot_1",
            description="Robot namespace to use for SLAM"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true"
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution([pkg_nav, "config", "slam_params.yaml"]),
            description="Path to slam_toolbox params"
        ),
        OpaqueFunction(function=_launch_slam),
    ])
