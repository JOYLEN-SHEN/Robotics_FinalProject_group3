"""
Spawn a single AGV robot into the running Gazebo simulation.

Usage:
  ros2 launch warehouse_gazebo spawn_single_robot.launch.py \
    robot_name:=robot_1 x:=-13.0 y:=7.0 yaw:=0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = FindPackageShare("warehouse_description")

    # ---- Arguments ----
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="robot_1",
        description="Unique robot name used as ROS2 namespace"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0",  description="Spawn X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0",  description="Spawn Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.1",  description="Spawn Z position")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw [rad]")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation clock"
    )

    robot_name    = LaunchConfiguration("robot_name")
    x             = LaunchConfiguration("x")
    y             = LaunchConfiguration("y")
    z             = LaunchConfiguration("z")
    yaw           = LaunchConfiguration("yaw")
    use_sim_time  = LaunchConfiguration("use_sim_time")

    # ---- Robot description (xacro -> URDF) ----
    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "agv_robot.urdf.xacro"])
    robot_description = ParameterValue(Command([
        "xacro ", xacro_file,
        " robot_name:=", robot_name,
        " robot_namespace:=", robot_name,
    ]), value_type=str)

    # ---- robot_state_publisher (namespaced) ----
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
            "publish_frequency": 50.0,
        }],
        # Remap /tf and /tf_static → relative so transforms land in /<robot>/tf
        # (nav2 nodes use the same remapping, so they share the same TF tree)
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output="screen",
    )

    # ---- Gazebo spawn entity ----
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        arguments=[
            "-topic",    ["/", robot_name, "/robot_description"],
            "-entity",   robot_name,
            "-x",        x,
            "-y",        y,
            "-z",        z,
            "-Y",        yaw,
            "-robot_namespace", robot_name,
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_name_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        use_sim_time_arg,
        robot_state_publisher,
        spawn_entity,
    ])
