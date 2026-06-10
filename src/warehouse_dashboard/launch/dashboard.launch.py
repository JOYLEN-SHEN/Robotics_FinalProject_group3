"""Launch the warehouse web dashboard node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="5000",
        description="HTTP port for the dashboard"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    dashboard_node = Node(
        package="warehouse_dashboard",
        executable="dashboard_node",
        name="warehouse_dashboard",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        port_arg,
        use_sim_time_arg,
        dashboard_node,
    ])
