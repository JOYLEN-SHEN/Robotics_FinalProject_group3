#!/usr/bin/env bash
#
# 实时跟踪多机器人仓库系统日志，自动高亮错误/警告。
# 用法：
#   ./scripts/tail_logs.sh              # 跟踪 simulation.log（推荐）
#   ./scripts/tail_logs.sh fleet         # 跟踪 fleet_manager 相关
#   ./scripts/tail_logs.sh nav2          # 跟踪 nav2 相关
#   ./scripts/tail_logs.sh errors        # 只看错误和警告
#   ./scripts/tail_logs.sh all           # 跟踪所有日志
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$REPO_ROOT/logs"

RED='\033[1;31m'; YELLOW='\033[1;33m'; GREEN='\033[1;32m'
CYAN='\033[1;36m'; BOLD='\033[1m'; DIM='\033[2m'; RESET='\033[0m'

# 创建一个临时合并日志（如果不存在）
MERGED="$LOG_DIR/_merged.log"
touch "$MERGED"

if [[ ! -d "$LOG_DIR" ]] || [[ -z "$(ls -A "$LOG_DIR" 2>/dev/null | grep -v _merged.log)" ]]; then
    echo -e "${YELLOW}⚠ 日志目录为空或不存在: $LOG_DIR${RESET}"
    echo -e "${DIM}提示: 先启动系统 (python3 scripts/start_all.py)${RESET}"
    exit 1
fi

# 构建高亮脚本（用 awk 给 ERROR/WARN 染色）
HIGHLIGHT='
BEGIN { ff=""; }
{
  line = $0;
  # 去除 ANSI 颜色码后匹配
  gsub(/\x1b\[[0-9;]*[mK]/, "", line);
  if (line ~ /\[(ERROR)\]|FATAL|Unhandled|Traceback|Invalid frame|Transform|Timed out|process has died/) {
    printf "\033[1;31m%s\033[0m\n", $0; ff="";
  } else if (line ~ /\[(WARN|WARNING)\]/) {
    printf "\033[1;33m%s\033[0m\n", $0; ff="";
  } else if (line ~ /\[(INFO)\]|started|launched|initialized|ready/) {
    # 信息类用 DIM 显示，避免刷屏
    if (length($0) > 200) $0 = substr($0, 1, 200) "...";
    printf "\033[2m%s\033[0m\n", $0; ff="";
  } else {
    print $0; ff="";
  }
  fflush();
}'

case "${1:-main}" in
    main|sim|simulation)
        LOG="$LOG_DIR/simulation.log"
        [[ -f "$LOG" ]] || { echo -e "${RED}✗ 找不到 $LOG${RESET}"; exit 1; }
        echo -e "${BOLD}${CYAN}跟踪主日志: $LOG${RESET}  ${DIM}(Ctrl+C 退出)${RESET}"
        tail -F "$LOG" | awk "$HIGHLIGHT"
        ;;
    fleet)
        echo -e "${BOLD}${CYAN}跟踪 Fleet Manager 日志${RESET}"
        tail -F "$LOG_DIR"/simulation.log 2>/dev/null \
            | grep --line-buffered -E "fleet_manager|Fleet|task" \
            | awk "$HIGHLIGHT"
        ;;
    nav2|nav)
        echo -e "${BOLD}${CYAN}跟踪 Nav2 日志${RESET}"
        tail -F "$LOG_DIR"/simulation.log 2>/dev/null \
            | grep --line-buffered -E "controller_server|planner_server|amcl|nav2|robot_[1-4]/" \
            | awk "$HIGHLIGHT"
        ;;
    errors|err)
        echo -e "${BOLD}${RED}只看错误/警告${RESET}  ${DIM}(Ctrl+C 退出)${RESET}"
        tail -F "$LOG_DIR"/simulation.log 2>/dev/null \
            | grep --line-buffered -E -i "error|warn|fail|traceback|invalid frame|timeout" \
            | awk "$HIGHLIGHT"
        ;;
    all)
        echo -e "${BOLD}${CYAN}跟踪所有日志文件${RESET}  ${DIM}(Ctrl+C 退出)${RESET}"
        # 合并所有 .log 文件（排除 _merged.log 自身）
        for f in "$LOG_DIR"/*.log; do
            [[ "$(basename "$f")" == "_merged.log" ]] && continue
            [[ -f "$f" ]] || continue
        done
        tail -F "$LOG_DIR"/*.log 2>/dev/null | awk "$HIGHLIGHT"
        ;;
    *)
        echo -e "${YELLOW}未知模式: $1${RESET}"
        echo -e "用法: $0 {main|fleet|nav2|errors|all}"
        exit 1
        ;;
esac
