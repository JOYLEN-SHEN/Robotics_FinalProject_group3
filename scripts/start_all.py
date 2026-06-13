#!/usr/bin/env python3
"""
一键启动多机器人仓库仿真系统（简化版）。

策略：使用项目自带的 full_simulation.launch.py（它已经集成了
Gazebo + 4x Nav2 + Fleet Manager + Dashboard + TF Relay + RViz），
并通过 --log-dir 把所有 ROS 日志重定向到 logs/ 目录方便查看。

用法：
    python3 scripts/start_all.py                  # 启动完整系统
    python3 scripts/start_all.py --headless       # 无 Gazebo GUI（无头模式）
    python3 scripts/start_all.py --no-rviz        # 不启动 RViz
    python3 scripts/start_all.py --keep-tmux      # 启动后不附加到 tmux
    python3 scripts/start_all.py --no-tmux        # 直接前台运行（不开新窗口）

输出位置：
    终端1 (Gazebo):     logs/gazebo.log
    终端2 (Nav2x4):     logs/nav2.log
    终端3 (Fleet):      logs/fleet.log
    终端4 (Dashboard):  logs/dashboard.log
    终端5 (TF relay):   logs/tf_relay.log
    终端6 (RViz):       logs/rviz.log
    ROS 内部日志:       /home/<user>/.ros/log/latest/

也可以在另一个终端用 scripts/tail_logs.sh 实时跟踪。
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR   = REPO_ROOT / "logs"
SESSION   = "warehouse"


# ==================== ANSI 颜色 ====================
class C:
    RED, YELLOW, GREEN, CYAN, BOLD, DIM, RESET = (
        "\033[1;31m", "\033[1;33m", "\033[1;32m",
        "\033[1;36m", "\033[1m", "\033[2m", "\033[0m",
    )


# ==================== 启动前检查 ====================

def step(title: str) -> None:
    print(f"{C.BOLD}{title}{C.RESET}", flush=True)


def ok(msg: str)   -> None: print(f"  {C.GREEN}✓{C.RESET} {msg}", flush=True)
def warn(msg: str) -> None: print(f"  {C.YELLOW}⚠{C.RESET} {msg}", flush=True)
def fail(msg: str) -> None: print(f"  {C.RED}✗{C.RESET} {msg}", flush=True)


def check_env() -> bool:
    step("[1/4] 检查 ROS2 环境")
    if not os.environ.get("ROS_DISTRO"):
        fail("ROS2 未 source")
        print(f"    {C.DIM}修复: source /opt/ros/humble/setup.bash{C.RESET}")
        return False
    ok(f"ROS_DISTRO = {os.environ['ROS_DISTRO']}")
    return True


def check_workspace() -> bool:
    step("[2/4] 检查工作区编译")
    install = REPO_ROOT / "install" / "setup.bash"
    if not install.exists():
        fail("工作区未编译 (install/setup.bash 缺失)")
        print(f"    {C.DIM}修复: colcon build --symlink-install{C.RESET}")
        return False
    # 校验关键包
    needed = ["warehouse_gazebo", "warehouse_navigation",
              "fleet_manager", "warehouse_dashboard", "warehouse_msgs"]
    missing = [p for p in needed if not (REPO_ROOT / "install" / p).exists()]
    if missing:
        warn(f"未编译包: {', '.join(missing)}")
        print(f"    {C.DIM}修复: colcon build --packages-select {' '.join(missing)} --symlink-install{C.RESET}")
        return False
    ok("工作区已编译，关键包齐全")
    return True


def check_map() -> bool:
    step("[3/4] 检查地图文件")
    pkg = REPO_ROOT / "install" / "warehouse_navigation" / "share" / "warehouse_navigation" / "maps"
    candidates = list(pkg.glob("*.yaml")) if pkg.exists() else []
    if not candidates:
        fail("找不到任何地图 yaml 文件")
        print(f"    {C.DIM}期望: {pkg}{C.RESET}")
        print(f"    {C.DIM}修复: 建图后保存到 src/warehouse_navigation/maps/warehouse_map.yaml 再编译{C.RESET}")
        return False
    ok(f"找到 {len(candidates)} 个地图: {', '.join(p.name for p in candidates)}")
    return True


def check_tools() -> bool:
    step("[4/4] 检查依赖工具")
    if not shutil.which("tmux"):
        warn("tmux 未安装（将退化为前台模式）")
        print(f"    {C.DIM}建议: sudo apt install -y tmux  （便于多窗口管理）{C.RESET}")
        return True
    ok("tmux 已安装")
    return True


# ==================== tmux 模式 ====================

def tmux_has() -> bool:
    return subprocess.run(["tmux", "has-session", "-t", SESSION],
                          capture_output=True).returncode == 0


def tmux_kill() -> None:
    if tmux_has():
        subprocess.run(["tmux", "kill-session", "-t", SESSION], capture_output=True)


def tmux_start(args: argparse.Namespace) -> None:
    """
    创建 3 个 tmux 窗口：
      0. main:   full_simulation.launch.py（统一启动所有东西）
      1. tail:   实时跟踪关键日志
      2. shell:  备用 shell
    """
    tmux_kill()
    LOG_DIR.mkdir(exist_ok=True)
    for f in LOG_DIR.glob("*.log"):
        f.unlink()

    env_setup = "source /opt/ros/humble/setup.bash && source install/setup.bash"

    # ---- 构造 launch 参数 ----
    launch_args = ["use_sim_time:=true"]
    if args.headless:
        launch_args.append("gui:=false")
    if args.no_rviz:
        launch_args.append("launch_rviz:=false")

    # ---- 窗口 0: 主仿真 ----
    main_cmd = (
        f"{env_setup} && "
        f"echo '{C.CYAN}═══ 多机器人仓库仿真 (Gazebo + Nav2x4 + Fleet + Dashboard + RViz) ═══{C.RESET}' && "
        f"echo '{C.DIM}完整启动需要 ~30 秒，请等待所有节点就绪{C.RESET}' && "
        f"echo '' && "
        f"ros2 launch warehouse_gazebo full_simulation.launch.py "
        f"{' '.join(launch_args)} 2>&1 | "
        f"stdbuf -oL sed -E 's/\\x1b\\[[0-9;]*[mK]//g' | "
        f"tee {LOG_DIR}/simulation.log"
    )
    subprocess.run([
        "tmux", "new-session", "-d", "-s", SESSION,
        "-n", "simulation", "-x", "200", "-y", "50",
    ], check=True)
    subprocess.run([
        "tmux", "send-keys", "-t", f"{SESSION}:0.0", main_cmd, "Enter",
    ], check=True)

    # ---- 窗口 1: 日志跟踪（按等级高亮） ----
    tail_cmd = (
        f"echo '{C.CYAN}═══ 实时日志跟踪（错误/警告高亮）═══{C.RESET}' && "
        f"echo '{C.DIM}用 Ctrl+B → 0/1/2 切换窗口{C.RESET}' && "
        f"echo '' && "
        f"tail -F {LOG_DIR}/simulation.log 2>/dev/null | "
        f"awk '{{"
        f"  if (/\\[(ERROR)\\]/) {{ printf \"\\033[1;31m%s\\033[0m\\n\", $0; fflush(); }}"
        f"  else if (/\\[(WARN|WARNING)\\]/) {{ printf \"\\033[1;33m%s\\033[0m\\n\", $0; fflush(); }}"
        f"  else if (/\\[(ERROR|WARN)\\]|Timeout|Invalid frame/) {{ printf \"\\033[1;31m%s\\033[0m\\n\", $0; fflush(); }}"
        f"  else print; fflush();"
        f"}}'"
    )
    subprocess.run([
        "tmux", "new-window", "-t", SESSION, "-n", "logs",
        "-x", "200", "-y", "50",
    ], check=True)
    subprocess.run([
        "tmux", "send-keys", "-t", f"{SESSION}:1", tail_cmd, "Enter",
    ], check=True)

    # ---- 窗口 2: 备用 shell ----
    shell_cmd = (
        f"{env_setup} && "
        f"echo '{C.CYAN}═══ 备用 shell (查看话题/节点/服务) ═══{C.RESET}' && "
        f"echo '{C.DIM}常用命令:{C.RESET}' && "
        f"echo '  ros2 topic list | grep -E \"fleet|task|scan\"' && "
        f"echo '  ros2 node list' && "
        f"echo '  ros2 run tf2_tools view_frames' && "
        f"echo '  ros2 service call /submit_task warehouse_msgs/srv/AssignTask ...' && "
        f"exec bash"
    )
    subprocess.run([
        "tmux", "new-window", "-t", SESSION, "-n", "shell",
        "-x", "200", "-y", "50",
    ], check=True)
    subprocess.run([
        "tmux", "send-keys", "-t", f"{SESSION}:2", shell_cmd, "Enter",
    ], check=True)

    # 选中第一个窗口
    subprocess.run(["tmux", "select-window", "-t", f"{SESSION}:0"], check=True)


# ==================== 前台模式（无 tmux） ====================

def foreground_start(args: argparse.Namespace) -> None:
    """无 tmux：直接前台运行 launch。"""
    env_setup = "source /opt/ros/humble/setup.bash && source install/setup.bash"
    LOG_DIR.mkdir(exist_ok=True)

    launch_args = ["use_sim_time:=true"]
    if args.headless:
        launch_args.append("gui:=false")
    if args.no_rviz:
        launch_args.append("launch_rviz:=false")

    cmd = (
        f"{env_setup} && "
        f"ros2 launch warehouse_gazebo full_simulation.launch.py "
        f"{' '.join(launch_args)} 2>&1 | "
        f"tee {LOG_DIR}/simulation.log"
    )
    os.execvp("bash", ["bash", "-c", cmd])


# ==================== 入口 ====================

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="一键启动多机器人仓库仿真系统",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  python3 scripts/start_all.py                   # 完整启动（Gazebo GUI + RViz）
  python3 scripts/start_all.py --headless        # 无 Gazebo GUI
  python3 scripts/start_all.py --no-tmux         # 直接前台运行
        """,
    )
    p.add_argument("--headless",  action="store_true",
                   help="无 Gazebo 图形界面（适合无显示器/远程）")
    p.add_argument("--no-rviz",   action="store_true",
                   help="不启动 RViz")
    p.add_argument("--no-tmux",   action="store_true",
                   help="不使用 tmux，直接前台运行")
    p.add_argument("--keep-tmux", action="store_true",
                   help="启动后不自动 attach 到 tmux 会话")
    return p.parse_args()


def banner(use_tmux: bool) -> None:
    print(f"""
{C.BOLD}{C.CYAN}╔════════════════════════════════════════════════════════════╗
║     多机器人仓库仿真系统  —  一键启动                        ║
╚════════════════════════════════════════════════════════════╝{C.RESET}

{C.BOLD}日志目录:{C.RESET}  {LOG_DIR}/
{C.BOLD}启动方式:{C.RESET}  {'tmux 多窗口' if use_tmux else '前台运行'}

{C.BOLD}完整启动时间:{C.RESET} ~30 秒（Gazebo 5s + Spawn 5s + Nav2 8s + Fleet 5s + Dashboard 2s）

{C.BOLD}关键提示:{C.RESET}
  • Gazebo 第一次启动会比较慢（物理引擎初始化）
  • 所有节点准备好后，浏览器打开 {C.GREEN}http://localhost:5000{C.RESET} 看 Dashboard
  • 报错时优先看 {LOG_DIR}/simulation.log（最完整）""")
    if use_tmux:
        print(f"""
{C.BOLD}tmux 操作:{C.RESET}
  {C.GREEN}Ctrl+B 然后按 0{C.RESET}  → 主仿真窗口（看实时启动日志）
  {C.GREEN}Ctrl+B 然后按 1{C.RESET}  → 日志跟踪窗口（错误/警告自动高亮）
  {C.GREEN}Ctrl+B 然后按 2{C.RESET}  → 备用 shell（查话题/发任务）
  {C.GREEN}Ctrl+B 然后按 d{C.RESET}  → 脱离 tmux（系统继续运行）
  {C.GREEN}python3 scripts/stop_all.py{C.RESET}  → 完全停止系统""")
    print()


def main() -> int:
    args = parse_args()

    print(f"{C.BOLD}启动前环境检查{C.RESET}\n", flush=True)
    if not all([check_env(), check_workspace(), check_map(), check_tools()]):
        print(f"\n{C.RED}环境检查未通过，请按提示修复后重试。{C.RESET}")
        return 1

    use_tmux = shutil.which("tmux") and not args.no_tmux
    banner(use_tmux)

    if use_tmux:
        tmux_start(args)
        print(f"{C.GREEN}✓ tmux 会话 '{SESSION}' 已创建{C.RESET}")
        print(f"{C.DIM}日志实时写入: {LOG_DIR}/simulation.log{C.RESET}\n")

        if args.keep_tmux:
            return 0
        os.execvp("tmux", ["tmux", "attach", "-t", SESSION])
    else:
        foreground_start(args)

    return 0


if __name__ == "__main__":
    sys.exit(main())
