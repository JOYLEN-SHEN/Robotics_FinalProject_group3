"""
Spawn all 4 AGV robots into the running Gazebo simulation.
Robot positions are loaded from robot_spawn_params.yaml.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Default spawn positions (also in robot_spawn_params.yaml)
ROBOTS = [
    {"name": "robot_1", "x": "-13.0", "y":  "7.0", "z": "0.1", "yaw": "0.0"},
    {"name": "robot_2", "x":  "13.0", "y":  "7.0", "z": "0.1", "yaw": "3.14159"},
    {"name": "robot_3", "x": "-13.0", "y": "-7.0", "z": "0.1", "yaw": "0.0"},
    {"name": "robot_4", "x":  "13.0", "y": "-7.0", "z": "0.1", "yaw": "3.14159"},
]


def generate_launch_description():
    pkg_gazebo = FindPackageShare("warehouse_gazebo")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation clock"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    spawn_launch = PathJoinSubstitution([pkg_gazebo, "launch", "spawn_single_robot.launch.py"])

    actions = [use_sim_time_arg]

    for robot in ROBOTS:
        spawn_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "robot_name":   robot["name"],
                "x":            robot["x"],
                "y":            robot["y"],
                "z":            robot["z"],
                "yaw":          robot["yaw"],
                "use_sim_time": use_sim_time,
            }.items(),
        )
        actions.append(spawn_action)

    return LaunchDescription(actions)
