"""Launch RViz2 for visualizing the AGV robot model."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("warehouse_description")

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot_1",
        description="Name/namespace of the robot"
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="true",
        description="Start joint_state_publisher_gui instead of joint_state_publisher"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "rviz_config.rviz"]),
        description="Path to RViz2 config file"
    )

    robot_name = LaunchConfiguration("robot_name")
    rviz_config = LaunchConfiguration("rviz_config")

    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "agv_robot.urdf.xacro"])
    robot_description = Command([
        "xacro ", xacro_file,
        " robot_name:=", robot_name,
        " robot_namespace:=", robot_name,
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        output="screen",
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        namespace=robot_name,
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        robot_name_arg,
        use_gui_arg,
        rviz_config_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
    ])
