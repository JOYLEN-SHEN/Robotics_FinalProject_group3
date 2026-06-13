#!/usr/bin/env bash
#
# 查看多机器人仓库系统状态。
# 显示：节点数、话题数、Dashboard 健康、TF 树、地图是否加载。
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 自动 source 环境（如果还没 source）
if [[ -z "${ROS_DISTRO:-}" ]]; then
    source /opt/ros/humble/setup.bash
    source "$REPO_ROOT/install/setup.bash"
fi

RED='\033[1;31m'; YELLOW='\033[1;33m'; GREEN='\033[1;32m'
CYAN='\033[1;36m'; BOLD='\033[1m'; DIM='\033[2m'; RESET='\033[0m'

print_section() {
    echo
    echo -e "${BOLD}${CYAN}═══ $1 ═══${RESET}"
}

check() {
    local name="$1"
    local cmd="$2"
    local result
    if result=$(eval "$cmd" 2>/dev/null) && [[ -n "$result" ]]; then
        echo -e "  ${GREEN}✓${RESET}  $name  ${DIM}$result${RESET}"
    else
        echo -e "  ${RED}✗${RESET}  $name  ${DIM}(未就绪)${RESET}"
    fi
}

print_section "节点状态"
NODES=$(ros2 node list 2>/dev/null | sort || echo "")
if [[ -n "$NODES" ]]; then
    for n in $NODES; do
        echo -e "  ${DIM}•${RESET}  $n"
    done
    echo
    echo -e "  ${BOLD}总计: $(echo "$NODES" | wc -l) 个节点${RESET}"
else
    echo -e "  ${RED}✗ 没有运行中的 ROS 节点（系统未启动）${RESET}"
    exit 0
fi

print_section "关键子系统"
check "Gazebo 仿真"   "ros2 node list | grep -c gz_sim | xargs -I {} echo '{} 个 gz_sim 节点'"
check "Nav2 控制器 (期望: 4)" "ros2 node list | grep -c controller_server | xargs -I {} echo '{} 个 controller_server'"
check "Nav2 Planner (期望: 4)" "ros2 node list | grep -c planner_server   | xargs -I {} echo '{} 个 planner_server'"
check "AMCL 定位 (期望: 4)"    "ros2 node list | grep -c amcl            | xargs -I {} echo '{} 个 amcl'"
check "Fleet Manager"         "ros2 node list | grep -c fleet_manager    | xargs -I {} echo '{} 个 fleet_manager'"
check "Dashboard 节点"        "ros2 node list | grep -c warehouse_dashboard | xargs -I {} echo '{} 个 warehouse_dashboard'"
check "TF Relay"              "ros2 node list | grep -c tf_relay         | xargs -I {} echo '{} 个 tf_relay'"
check "RViz2"                 "ros2 node list | grep -c rviz2            | xargs -I {} echo '{} 个 rviz2'"

print_section "Dashboard HTTP"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" --max-time 2 http://localhost:5000/api/health 2>/dev/null || echo "000")
if [[ "$HTTP_CODE" == "200" ]]; then
    BODY=$(curl -s --max-time 2 http://localhost:5000/api/health 2>/dev/null)
    echo -e "  ${GREEN}✓${RESET}  Dashboard 运行中 (HTTP 200)  ${DIM}$BODY${RESET}"
    echo -e "  ${BOLD}访问: ${GREEN}http://localhost:5000${RESET}"
else
    echo -e "  ${RED}✗${RESET}  Dashboard 未运行（HTTP $HTTP_CODE）"
fi

print_section "关键话题"
TOPICS=$(ros2 topic list 2>/dev/null | grep -E "fleet_status|task_requests|task_results|/cmd_vel|/scan" | sort)
if [[ -n "$TOPICS" ]]; then
    for t in $TOPICS; do
        # 查最近一次发布
        INFO=$(ros2 topic info "$t" 2>/dev/null | grep -E "Publisher count" | head -1)
        echo -e "  ${DIM}•${RESET}  $t  ${DIM}$INFO${RESET}"
    done
else
    echo -e "  ${YELLOW}⚠${RESET}  找不到关键话题"
fi

print_section "TF 关键帧"
TF_CHECK=$(ros2 run tf2_ros tf2_echo map robot_1/base_link --timeout 1 2>&1 | head -3 || true)
if echo "$TF_CHECK" | grep -q "Translation\|At time"; then
    echo -e "  ${GREEN}✓${RESET}  map → robot_1/base_link TF 链有效"
else
    echo -e "  ${RED}✗${RESET}  map → robot_1/base_link TF 链无效或不存在"
    echo -e "      ${DIM}可能原因: SLAM/AMCL 未初始化, 或机器人 spawn 失败${RESET}"
fi

print_section "完成"
