"""Launch the Fleet Manager node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_fleet = FindPackageShare("fleet_manager")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([pkg_fleet, "config", "fleet_config.yaml"]),
        description="Fleet manager config YAML",
    )
    strategy_arg = DeclareLaunchArgument(
        "allocation_strategy", default_value="greedy",
        description="Task allocation strategy: 'greedy' or 'hungarian'",
    )

    fleet_manager_node = Node(
        package="fleet_manager",
        executable="fleet_manager_node",
        name="fleet_manager",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "allocation_strategy": LaunchConfiguration("allocation_strategy"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_arg,
        strategy_arg,
        fleet_manager_node,
    ])
