#!/usr/bin/env bash
#
# 通过 ROS2 service 向 Fleet Manager 发布搬运任务。
#
# 用法：
#   ./scripts/submit_task.sh LOADING_DOCK UNLOADING_DOCK [PRIORITY]
#
# 示例：
#   ./scripts/submit_task.sh loading_dock_1 unloading_dock_1
#   ./scripts/submit_task.sh loading_dock_2 unloading_dock_2 2     # 优先级 2=高
#
# 坐标来源（src/fleet_manager/config/warehouse_graph.yaml）:
#   loading_dock_1:   (-14,  3)
#   loading_dock_2:   (-14, -3)
#   unloading_dock_1: ( 14,  3)
#   unloading_dock_2: ( 14, -3)
#   charging_station: (  0,  0)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
    source /opt/ros/humble/setup.bash
    source "$REPO_ROOT/install/setup.bash"
fi

# 区域坐标（与 fleet_manager 配置一致）
declare -A PICKUP_X=(
    [loading_dock_1]="-14.0"  [loading_dock_2]="-14.0"
    [unloading_dock_1]="14.0"  [unloading_dock_2]="14.0"
    [charging_station]="0.0"
)
declare -A PICKUP_Y=(
    [loading_dock_1]="3.0"    [loading_dock_2]="-3.0"
    [unloading_dock_1]="3.0"   [unloading_dock_2]="-3.0"
    [charging_station]="0.0"
)

PICK="${1:-}"
DROP="${2:-}"
PRIORITY="${3:-1}"

if [[ -z "$PICK" ]] || [[ -z "$DROP" ]]; then
    echo "用法: $0 PICKUP_ZONE DROPOFF_ZONE [PRIORITY]"
    echo "示例: $0 loading_dock_1 unloading_dock_1 2"
    echo
    echo "可用区域:"
    echo "  ${!PICKUP_X[*]}"
    exit 1
fi

if [[ -z "${PICKUP_X[$PICK]:-}" ]] || [[ -z "${PICKUP_X[$DROP]:-}" ]]; then
    echo "✗ 未知区域: '$PICK' 或 '$DROP'"
    echo "可用区域: ${!PICKUP_X[*]}"
    exit 1
fi

PX="${PICKUP_X[$PICK]}"
PY="${PICKUP_Y[$PICK]}"
DX="${PICKUP_X[$DROP]}"
DY="${PICKUP_Y[$DROP]}"

echo "→ 提交任务: $PICK ($PX, $PY) → $DROP ($DX, $DY)  优先级=$PRIORITY"

ros2 service call /submit_task warehouse_msgs/srv/AssignTask \
    "{
        pickup_zone:  '$PICK',
        dropoff_zone: '$DROP',
        pickup_x:     $PX,
        pickup_y:     $PY,
        dropoff_x:    $DX,
        dropoff_y:    $DY,
        priority:     $PRIORITY,
        timeout_sec:  120.0
    }" || { echo "✗ 提交失败（确认 fleet_manager 已启动）"; exit 1; }
