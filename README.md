# Multi-Robot Warehouse — 多AGV动态调度系统

基于 ROS2 Humble 的多AGV智能仓储系统。四台AGV小车在Gazebo仿真环境中完成货物自动搬运，
由 Fleet Manager 统一调度，支持动态调度策略自适应切换、负荷差异研究与简单冲突规避。

## 核心特性

- **多AGV动态调度**：根据任务负荷实时自动切换最优调度策略
- **任务负荷差异研究**：支持任务密集度、车辆负载、峰值检测等多维度分析
- **简化冲突规避**：替代复杂CBS算法，仅保留主动避让与死锁纠错
- **高可扩展架构**：模块化设计，支持扩展至更多车辆与更复杂仓库布局

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                         Fleet Manager (动态调度核心)                │
│  ┌────────────────┐  ┌────────────────┐  ┌──────────────────┐   │
│  │FleetLoadAnalyzer│  │AdaptiveTask    │  │SimpleConflict     │   │
│  │(负荷感知分析)   │→ │Allocator       │← │Resolver          │   │
│  │• 负荷等级检测   │  │(自适应分配)    │  │(简单冲突规避)    │   │
│  │• 策略推荐       │  │• 5种调度策略   │  │• 距离检测+避让   │   │
│  │• 任务密集度     │  │• 多因子评分    │  │• 死锁检测+纠错   │   │
│  └────────────────┘  └────────────────┘  └──────────────────┘   │
│                              ↑                                    │
│  ┌────────────────┐  ┌────────────────┐  ┌──────────────────┐   │
│  │ RobotStateMgr   │  │  TaskQueue     │  │  WarehouseGraph   │   │
│  │ (状态+电池管理) │  │ (优先级队列)   │  │  (拓扑地图)       │   │
│  └────────────────┘  └────────────────┘  └──────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
                               │ Nav2 Actions
       ┌────────────┬──────────┼──────────┬────────────┐
       ▼            ▼          ▼          ▼            ▼
  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐
  │ Robot 1 │ │ Robot 2 │ │ Robot 3 │ │ Robot 4 │ │ Robot N │
  │ Nav2+   │ │ Nav2+   │ │ Nav2+   │ │ Nav2+   │ │ Nav2+   │
  │ SLAM    │ │ SLAM    │ │ SLAM    │ │ SLAM    │ │ SLAM    │
  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘
       └────────────┴────────────┴────────────┴────────────┘
                              ▼
                     ┌──────────────────┐
                     │  Gazebo Sim      │
                     │  (Warehouse +    │
                     │   AGV robots)    │
                     └──────────────────┘
                              │
                     ┌──────────────────┐
                     │  Web Dashboard   │
                     │  Flask+SocketIO  │
                     │  localhost:5000  │
                     └──────────────────┘
```

## 调度策略引擎

系统支持 **5 种调度策略**，根据负荷等级自动切换：

| 策略 | 触发条件 | 适用场景 | 算法特点 |
|------|---------|---------|---------|
| **轮询分配** (Round Robin) | LOW 负荷 | 低峰期、任务稀疏 | 任务均摊各车，无偏好 |
| **贪心分配** (Greedy) | MODERATE 负荷 | 中等任务量 | 综合距离+电量+优先级评分 |
| **全局最优** (Hungarian) | HIGH 负荷 | 任务密集、高峰前期 | O(n³) 全局最优匹配 |
| **紧急模式** (Emergency) | PEAK 负荷 | 突发峰值、紧急任务 | 电池优先+截止时间惩罚 |
| **负载均衡** (Load Balanced) | 任意 | 多车协同、长期运行 | 基于负荷画像防止过载 |

### 负荷等级判定

```
负荷得分 = 0.4×任务密集度分 + 0.3×机器人利用率分 + 0.3×积压数量分

LOW      → 轮询分配    (任务稀疏，机器人空闲)
MODERATE → 贪心分配    (任务量适中)
HIGH     → 全局最优    (任务密集，需最优匹配)
PEAK     → 紧急模式    (超高峰值，系统过载)
```

## 任务分配评分函数

```
score = w_batt × 电池因子
      + w_dist × 邻近因子
      - w_workload × 工作量因子
      + w_priority × 优先级因子
      + w_deadline × 截止时间因子

电池因子：电量越低越不适合接新任务（<40% 降权，<20% 禁用）
邻近因子：距离 pickup 越近得分越高（归一化 0~1）
工作量因子：已完成任务越多，得分越低（负载均衡）
优先级因子：CRITICAL=2.0, HIGH=1.5, NORMAL=1.0, LOW=0.5
截止时间因子：即将到期任务强制分配
```

## 简单冲突规避机制

完全替代原有 CBS 算法，仅保留运行时冲突处理：

1. **实时距离检测**：每 0.5s 检测所有车辆对间距
2. **预警避让**（>1.0m）：低优先级车辆减速让行
3. **紧急停车**（<0.7m）：立即停车等待
4. **死锁检测**：位置历史停滞超过阈值时触发重规划
5. **交汇点管理**：狭窄通道令牌机制，防止对向死锁

## Package 结构

| Package | 说明 |
|---------|------|
| `warehouse_msgs` | ROS2 自定义消息、服务、动作 |
| `warehouse_description` | AGV URDF/Xacro、Gazebo 世界文件 |
| `warehouse_gazebo` | 仿真启动文件 |
| `warehouse_navigation` | Nav2 + SLAM 参数配置 |
| `fleet_manager` | **核心：动态调度、负荷分析、冲突规避** |
| `warehouse_dashboard` | Flask+SocketIO Web 可视化面板 |

## 快速启动

### 前置依赖
- ROS2 Humble
- Gazebo Classic 11
- `ros-humble-navigation2`, `ros-humble-slam-toolbox`

```bash
# 安装依赖
cd ~/multi_robot_warehouse_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install flask flask-socketio eventlet scipy numpy

# 编译
colcon build --symlink-install
source install/setup.bash

# 启动完整仿真
ros2 launch warehouse_gazebo full_simulation.launch.py
```

### 发送任务

```bash
# 通过 Topic
ros2 topic pub /task_requests warehouse_msgs/msg/TaskRequest \
  '{task_id: "task_001", task_type: 0, pickup_zone: "loading_dock_1",
    dropoff_zone: "unloading_dock_1", priority: 2,
    pickup_location: {position: {x: -14.0, y: 3.0, z: 0.0}},
    dropoff_location: {position: {x: 14.0, y: 3.0, z: 0.0}}}'

# 通过 Service
ros2 service call /assign_task warehouse_msgs/srv/AssignTask \
  '{task: {task_id: "task_002", task_type: 0, priority: 1,
    pickup_location: {position: {x: -14.0, y: -3.0, z: 0.0}},
    dropoff_location: {position: {x: 14.0, y: -3.0, z: 0.0}}}}'
```

## 仓储布局

```
W=30m, H=20m
    装货区              货架区 (4×5)          卸货区
    LD1(-14,3)    R1 R2 R3 R4              UD1(14,3)
    LD2(-14,-3)   rows at x=-10,-4,4,10   UD2(14,-3)

    充电站（四角）:
    CS1(-13,9)  CS2(13,9)  CS3(-13,-9)  CS4(13,-9)
```

## Topics 参考

```
/fleet_status              warehouse_msgs/FleetStatus  @ 1Hz
/task_requests             warehouse_msgs/TaskRequest
/task_results              warehouse_msgs/TaskResult
/robot_X/scan              sensor_msgs/LaserScan
/robot_X/odom              nav_msgs/Odometry
/robot_X/cmd_vel           geometry_msgs/Twist

Services:
/assign_task               warehouse_msgs/srv/AssignTask
/cancel_task              warehouse_msgs/srv/CancelTask
/get_robot_status          warehouse_msgs/srv/GetRobotStatus
```
