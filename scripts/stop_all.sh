#!/usr/bin/env bash
#
# 一键停止多机器人仓库仿真系统。
# 作用：杀掉 tmux 会话 + 所有 ros2 节点 + Gazebo 残留进程。
#
set -euo pipefail

SESSION="warehouse"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

RED='\033[1;31m'; YELLOW='\033[1;33m'; GREEN='\033[1;32m'; BOLD='\033[1m'; RESET='\033[0m'

echo -e "${BOLD}正在停止多机器人仓库仿真系统...${RESET}"

# 1. 杀掉 tmux 会话
if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
    echo -e "  ${GREEN}✓${RESET} tmux 会话 '$SESSION' 已关闭"
else
    echo -e "  ${YELLOW}ℹ${RESET}  tmux 会话 '$SESSION' 不存在（可能未启动）"
fi

# 2. 杀掉所有 ros2 节点（防御性）
ROS_PIDS=$(pgrep -f "ros2 launch" || true)
if [[ -n "$ROS_PIDS" ]]; then
    echo -e "  ${YELLOW}ℹ${RESET}  清理残留 ros2 launch 进程: $ROS_PIDS"
    pkill -f "ros2 launch" 2>/dev/null || true
    sleep 1
fi

# 3. 杀掉 Gazebo 残留（gzsclient, gzserver, ruby）
GZ_PIDS=$(pgrep -f "gz sim|gzserver|gzclient|ignition\.gazebo" || true)
if [[ -n "$GZ_PIDS" ]]; then
    echo -e "  ${YELLOW}ℹ${RESET}  清理 Gazebo 残留: $GZ_PIDS"
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
fi

# 4. 杀掉 rviz2
RVIZ_PIDS=$(pgrep -f "rviz2" || true)
if [[ -n "$RVIZ_PIDS" ]]; then
    echo -e "  ${YELLOW}ℹ${RESET}  清理 RViz 进程: $RVIZ_PIDS"
    pkill -9 -f "rviz2" 2>/dev/null || true
fi

# 5. 杀掉可能的 nav2 / fleet / dashboard 残留
OTHER_PIDS=$(pgrep -f "warehouse_navigation|fleet_manager|warehouse_dashboard|nav2_bringup" || true)
if [[ -n "$OTHER_PIDS" ]]; then
    echo -e "  ${YELLOW}ℹ${RESET}  清理其他仓库进程: $OTHER_PIDS"
    pkill -9 -f "warehouse_navigation" 2>/dev/null || true
    pkill -9 -f "fleet_manager" 2>/dev/null || true
    pkill -9 -f "warehouse_dashboard" 2>/dev/null || true
    pkill -9 -f "nav2_bringup" 2>/dev/null || true
fi

sleep 1
echo -e "${GREEN}✓ 系统已停止${RESET}"
