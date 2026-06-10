"""
Save the SLAM-built warehouse map using nav2_map_server's map_saver_cli.

Usage (while SLAM is running and publishing the map topic):
  ros2 launch warehouse_navigation save_map.launch.py

Optional arguments:
  robot_name:=robot_1          Which robot's map topic to subscribe to
  map_name:=warehouse_map      Output filename (no extension)
  output_dir:=<path>           Directory to write .pgm and .yaml files

With --symlink-install, the default output_dir points to the source maps/
directory so the saved map is immediately available to Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare("warehouse_navigation")

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot_1",
        description="Robot whose /robot_N/map topic to subscribe to"
    )
    map_name_arg = DeclareLaunchArgument(
        "map_name", default_value="warehouse_map",
        description="Output map filename prefix (no extension)"
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=PathJoinSubstitution([pkg_nav, "maps"]),
        description="Directory where .pgm and .yaml files are written"
    )

    robot_name = LaunchConfiguration("robot_name")
    map_name   = LaunchConfiguration("map_name")
    output_dir = LaunchConfiguration("output_dir")

    save_map = ExecuteProcess(
        cmd=[
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-t", ["/", robot_name, "/map"],
            "-f", [output_dir, "/", map_name],
            "--occ", "0.65",
            "--free", "0.25",
            "--ros-args",
            "-p", "use_sim_time:=true",
            "-p", "save_map_timeout:=10.0",
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_name_arg,
        map_name_arg,
        output_dir_arg,
        save_map,
    ])
