"""
Full simulation launch: Gazebo world + 4 robots + Nav2 + Fleet Manager + RViz2.
This is the main entry point for the complete system.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo     = FindPackageShare("warehouse_gazebo")
    pkg_navigation = FindPackageShare("warehouse_navigation")
    pkg_fleet      = FindPackageShare("fleet_manager")
    pkg_dashboard  = FindPackageShare("warehouse_dashboard")

    # ---- Arguments ----
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true", description="Launch Gazebo GUI"
    )
    rviz_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Launch RViz2"
    )
    nav2_arg = DeclareLaunchArgument(
        "launch_nav2", default_value="true", description="Launch Nav2 navigation"
    )
    fleet_arg = DeclareLaunchArgument(
        "launch_fleet", default_value="true", description="Launch Fleet Manager"
    )
    dashboard_arg = DeclareLaunchArgument(
        "launch_dashboard", default_value="true", description="Launch web dashboard"
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([pkg_navigation, "maps", "warehouse_map.yaml"]),
        description="Full path to the pre-built warehouse map yaml"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_navigation, "config", "rviz_multi_robot.rviz"]),
        description="Path to RViz2 config file"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui          = LaunchConfiguration("gui")

    # ---- 1. Gazebo world ----
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, "launch", "warehouse_world.launch.py"])
        ),
        launch_arguments={"gui": gui, "use_sim_time": use_sim_time}.items(),
    )

    # ---- 2. Spawn robots (delayed to let Gazebo initialize) ----
    spawn_robots = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_gazebo, "launch", "spawn_multi_robot.launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        ],
    )

    # ---- 3. Nav2 for all robots (delayed further) ----
    nav2_bringup = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_navigation, "launch", "multi_nav2_bringup.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map":          LaunchConfiguration("map"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("launch_nav2")),
            )
        ],
    )

    # ---- 4. Fleet manager (after Nav2) ----
    fleet_manager = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_fleet, "launch", "fleet_manager.launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(LaunchConfiguration("launch_fleet")),
            )
        ],
    )

    # ---- 5. Dashboard (after fleet manager) ----
    dashboard = TimerAction(
        period=22.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_dashboard, "launch", "dashboard.launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(LaunchConfiguration("launch_dashboard")),
            )
        ],
    )

    # ---- 6. TF relay: bridge /robot_N/tf → /tf so RViz2 can see transforms ----
    tf_relay = Node(
        package="warehouse_navigation",
        executable="tf_relay.py",
        name="tf_relay",
        output="screen",
    )

    # ---- 7. RViz2 (launched immediately, waits for topics automatically) ----
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        nav2_arg,
        fleet_arg,
        dashboard_arg,
        map_arg,
        rviz_config_arg,
        gazebo_world,
        spawn_robots,
        nav2_bringup,
        tf_relay,
        fleet_manager,
        dashboard,
        rviz2_node,
    ])
