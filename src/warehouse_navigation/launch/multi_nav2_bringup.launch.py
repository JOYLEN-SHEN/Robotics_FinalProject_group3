"""
Launch Nav2 for all 4 warehouse robots with individual namespaces.

Each robot gets its own Nav2 stack using a per-robot params file generated
at launch time (via OpaqueFunction) with correct TF frame names.

The URDF publishes namespaced TF frames (e.g. robot_1/base_link, robot_1/odom).
Nav2 frame parameters must match these global frame names exactly — they are NOT
automatically namespaced by ROS2. OpaqueFunction patches the base nav2_params.yaml
for each robot before launching the Nav2 stack.
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction,
    IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


ROBOT_NAMES = ["robot_1", "robot_2", "robot_3", "robot_4"]

# Spawn positions from robot_spawn_params.yaml — used for AMCL initial pose
ROBOT_POSES = {
    "robot_1": (-13.0,  7.0, 0.0),
    "robot_2": ( 13.0,  7.0, 3.14159),
    "robot_3": (-13.0, -7.0, 0.0),
    "robot_4": ( 13.0, -7.0, 3.14159),
}


def _patch_nav2_params(base_params_path: str, robot_name: str, map_file: str = "") -> str:
    """
    Read the base nav2_params.yaml, substitute per-robot TF frame names,
    and write the result to a temp file. Returns the temp file path.

    Patched keys:
      amcl               → base_frame_id, odom_frame_id, initial pose
      bt_navigator       → robot_base_frame
      local_costmap      → robot_base_frame, global_frame (= odom frame)
      global_costmap     → robot_base_frame
      behavior_server    → robot_base_frame, global_frame (= odom frame)
    """
    with open(base_params_path, "r") as f:
        params = yaml.safe_load(f)

    base_frame = f"{robot_name}/base_link"
    odom_frame  = f"{robot_name}/odom"
    x, y, yaw  = ROBOT_POSES[robot_name]

    # AMCL localization frames + automatic initial pose from spawn position
    if "amcl" not in params:
        params["amcl"] = {"ros__parameters": {}}
    amcl = params["amcl"]["ros__parameters"]
    amcl["base_frame_id"]    = base_frame
    amcl["odom_frame_id"]    = odom_frame
    amcl["set_initial_pose"] = True
    amcl["initial_pose"]     = {"x": x, "y": y, "z": 0.0, "yaw": yaw}

    # BT Navigator
    params["bt_navigator"]["ros__parameters"]["robot_base_frame"] = base_frame

    # Absolute scan topic: costmap nodes run in /robot_N/local_costmap (or global_costmap)
    # namespace, so relative "scan" would resolve to /robot_N/local_costmap/scan — wrong.
    # Use absolute topic so both costmaps subscribe to the actual lidar.
    scan_topic = f"/{robot_name}/scan"

    # Local costmap: rolls in odom frame, base is the robot frame
    lc = params["local_costmap"]["local_costmap"]["ros__parameters"]
    lc["robot_base_frame"] = base_frame
    lc["global_frame"]     = odom_frame
    lc["voxel_layer"]["scan"]["topic"] = scan_topic

    # Global costmap: plans in map frame, base is the robot frame
    gc = params["global_costmap"]["global_costmap"]["ros__parameters"]
    gc["robot_base_frame"] = base_frame
    gc["obstacle_layer"]["scan"]["topic"] = scan_topic

    # Recovery behaviors operate in odom frame
    bs = params["behavior_server"]["ros__parameters"]
    bs["robot_base_frame"] = base_frame
    bs["global_frame"]     = odom_frame

    # Pre-fill yaml_filename so map_server gets the map even if RewrittenYaml fails
    if map_file:
        if "map_server" not in params:
            params["map_server"] = {"ros__parameters": {}}
        params["map_server"]["ros__parameters"]["yaml_filename"] = map_file

    # Write to /tmp with a fixed name (overwritten on each launch — harmless)
    tmp_path = os.path.join(tempfile.gettempdir(), f"warehouse_nav2_{robot_name}.yaml")
    with open(tmp_path, "w") as f:
        yaml.dump(params, f, default_flow_style=False)

    return tmp_path


def _launch_nav2_per_robot(context, *args, **kwargs):
    """
    OpaqueFunction body: resolves LaunchConfiguration values to strings,
    generates per-robot param files, and returns GroupAction launch entities.

    File paths (map, params_file) are resolved directly via
    get_package_share_directory to avoid PathJoinSubstitution default-value
    resolution issues when accessed through LaunchConfiguration.perform().
    """
    use_sim_time = context.perform_substitution(LaunchConfiguration("use_sim_time"))

    pkg_nav  = get_package_share_directory("warehouse_navigation")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # Resolve map path: use explicit arg if provided, else default
    map_file_arg = context.perform_substitution(LaunchConfiguration("map"))
    map_file = map_file_arg if map_file_arg else os.path.join(pkg_nav, "maps", "warehouse_map.yaml")

    # Resolve params path: always use package-relative default for robustness
    base_params = os.path.join(pkg_nav, "config", "nav2_params.yaml")

    nav2_launch_path = os.path.join(pkg_nav2, "launch", "bringup_launch.py")

    actions = []
    for robot_name in ROBOT_NAMES:
        robot_params_file = _patch_nav2_params(base_params, robot_name, map_file)

        nav2_group = GroupAction([
            # NOTE: Do NOT add PushRosNamespace here — nav2_bringup already pushes
            # the namespace when use_namespace=true. Double-pushing causes nodes to
            # land at /robot_N/robot_N/... which breaks parameter lookup.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    "namespace":       robot_name,
                    "use_namespace":   "true",
                    # nav2 bringup uses PythonExpression(['not ', x]) in several
                    # launch files — these args must be Python-style booleans
                    # ("False"/"True"), not YAML-style ("false"/"true")
                    "slam":            "False",
                    "map":             map_file,
                    "use_sim_time":    use_sim_time,
                    "params_file":     robot_params_file,
                    "autostart":       "true",
                    "use_composition": "False",
                    "use_respawn":     "False",
                }.items(),
            ),
        ])
        actions.append(nav2_group)

    return actions


def generate_launch_description():
    pkg_nav = FindPackageShare("warehouse_navigation")

    # DeclareLaunchArgument MUST come before OpaqueFunction in the list
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([pkg_nav, "maps", "warehouse_map.yaml"]),
        description="Full path to warehouse map yaml file"
    )
    nav2_params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg_nav, "config", "nav2_params.yaml"]),
        description="Base Nav2 parameters file (patched per-robot at launch time)"
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        nav2_params_arg,
        OpaqueFunction(function=_launch_nav2_per_robot),
    ])
