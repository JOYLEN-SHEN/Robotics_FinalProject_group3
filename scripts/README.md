# 多机器人仓库 — 启动脚本

简化整套系统的启动、停止、状态查看和任务发布。

## 5 个核心脚本

| 脚本 | 作用 |
|------|------|
| `start_all.py` | **一键启动**完整系统（tmux 多窗口） |
| `stop_all.sh`  | **一键停止**系统（清理所有残留进程） |
| `status.sh`    | 查看系统**运行状态**（节点、话题、Dashboard、TF） |
| `tail_logs.sh` | **实时跟踪**日志（错误/警告自动高亮） |
| `submit_task.sh` | 通过 service **发布搬运任务** |

## 快速上手

```bash
# 1. 启动（自动打开 tmux 3 窗口：仿真/日志/备用 shell）
python3 scripts/start_all.py

# 2. 在新终端查看状态
./scripts/status.sh

# 3. 在新终端跟踪错误
./scripts/tail_logs.sh errors

# 4. 提交一个搬运任务
./scripts/submit_task.sh loading_dock_1 unloading_dock_1

# 5. 浏览器看 Dashboard
# http://localhost:5000

# 6. 完全停止
python3 scripts/stop_all.sh
```

## 启动模式

```bash
# 默认：完整启动（Gazebo GUI + RViz + Dashboard + Fleet）
python3 scripts/start_all.py

# 无 Gazebo 图形界面（适合无显示器或远程 SSH）
python3 scripts/start_all.py --headless

# 不启动 RViz
python3 scripts/start_all.py --no-rviz

# 不用 tmux（直接前台跑，Ctrl+C 结束一切）
python3 scripts/start_all.py --no-tmux

# 启动后不自动 attach 到 tmux
python3 scripts/start_all.py --keep-tmux
```

## tmux 窗口布局

启动后有 3 个窗口：

| 编号 | 名称 | 内容 |
|------|------|------|
| `0`  | simulation | 主仿真（Gazebo+Nav2+Fleet+Dashboard+RViz 统一日志） |
| `1`  | logs       | 错误/警告高亮实时跟踪 |
| `2`  | shell      | 备用 shell（查话题/发任务） |

**快捷键**：
- `Ctrl+B` 然后按 `0/1/2` 切换窗口
- `Ctrl+B` 然后按 `d` 脱离 tmux（系统继续运行）
- `Ctrl+C` 在某窗口退出当前命令（窗口保留）

## 跟踪日志

```bash
./scripts/tail_logs.sh              # 主日志（默认）
./scripts/tail_logs.sh main         # 同上
./scripts/tail_logs.sh fleet        # 只看 Fleet Manager
./scripts/tail_logs.sh nav2         # 只看 Nav2
./scripts/tail_logs.sh errors       # 只看错误/警告
./scripts/tail_logs.sh all          # 所有日志文件
```

错误显示为**红色**，警告显示为**黄色**，普通 INFO 显示为**灰色**（避免刷屏）。

## 发布任务

```bash
./scripts/submit_task.sh loading_dock_1 unloading_dock_1      # 默认优先级 1
./scripts/submit_task.sh loading_dock_2 unloading_dock_2 2     # 优先级 2=高
./scripts/submit_task.sh loading_dock_1 unloading_dock_2 3     # 优先级 3=紧急
```

优先级：`1=NORMAL`, `2=HIGH`, `3=URGENT`

或者用 dashboard 的 Web 界面 http://localhost:5000 直接点。

## 故障排查

| 现象 | 排查 |
|------|------|
| Gazebo 启动卡住 | 等 30s，物理引擎初始化慢；或者 `killall -9 gz sim` 后重试 |
| Nav2 报 `Invalid frame "map"` | 多 nav2 没全部起来；等 30s 后再发任务 |
| Dashboard 打不开 | `./scripts/status.sh` 看 Dashboard 节点是否在；5000 端口被占用？ |
| fleet_manager 报 `ParameterAlreadyDeclared` | 没重新编译；先 `colcon build` |
| AMCL 机器人飘 | 给 rviz 里手动设 initial pose，或者等粒子收敛 |
| 一切正常但任务不分配 | 看 fleet_manager 日志：`./scripts/tail_logs.sh fleet` |

## 日志位置

- **统一日志**: `logs/simulation.log` （所有节点合并）
- **ROS2 内部日志**: `~/.ros/log/latest/`
- **Gazebo 状态**: `~/.gazebo/log/`
