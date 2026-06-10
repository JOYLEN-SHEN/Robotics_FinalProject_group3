# 多AGV仓储机器人动态调度仿真系统

## 项目环境配置与运行操作说明文档

---

**文档版本**：v2.0（适配多AGV动态调度重构版）
**适用项目**：基于ROS2 Humble + Gazebo Classic的多AGV仓储机器人动态调度仿真系统
**编写目的**：为毕业设计答辩提供完整、可复现的运行环境配置与操作指南
**难度定位**：零基础可复现

---

## 一、运行环境介绍

### 1.1 硬件运行环境说明

本项目为纯软件仿真项目，无需真实机器人硬件即可完整运行。所有仿真均在虚拟机或物理机中通过Gazebo物理引擎完成。

| 硬件项目 | 最低配置 | 推荐配置 |
|---------|---------|---------|
| CPU | Intel i5 第8代（4核） | Intel i7 第10代（8核）或以上 |
| 内存 | 8 GB RAM | 16 GB RAM |
| 磁盘空间 | 60 GB 可用空间 | 100 GB SSD |
| 显卡 | NVIDIA GTX 1050（2GB） | NVIDIA GTX 1660（6GB）或更高 |
| 显示器 | 1920×1080 分辨率 | 2560×1440 或双屏 |

> **说明**：若使用VMware虚拟机运行，建议分配至少4核CPU、8GB内存，并启用"虚拟化Intel VT-x/EPT"选项以获得更好的仿真性能。虚拟机中Gazebo性能会有所下降，建议优先使用双系统或物理机运行。

### 1.2 软件运行环境

#### 1.2.1 操作系统

| 操作系统 | 版本要求 | 说明 |
|---------|---------|------|
| **Ubuntu Linux** | 22.04（Jammy LTS） | **首选推荐**，ROS2 Humble 官方支持 |
| VMware 虚拟机 | Ubuntu 22.04 镜像 | 配置NAT网络，分配4核+8GB |
| Windows + WSL2 | Ubuntu 22.04（WSLg） | 可运行ROS2但Gazebo支持有限 |

> **重要**：ROS2 Humble 仅支持 Ubuntu 22.04 和 Windows 10/11，不支持 Ubuntu 20.04。macOS 下无官方支持。本文档以 Ubuntu 22.04 为标准环境进行说明。

#### 1.2.2 核心软件版本

| 软件 | 版本 | 说明 |
|------|------|------|
| **ROS2 Humble** | 22.04.x | 机器人操作系统核心框架 |
| **Gazebo Classic** | 11.x | 物理仿真引擎（非Gazebo Fortress） |
| **Python** | 3.8 ~ 3.10 | ROS2节点开发语言 |
| **Navigation2** | Humble 兼容版 | 多机器人导航框架 |
| **SLAM Toolbox** | Humble 兼容版 | 激光SLAM建图工具 |
| **CMake** | ≥ 3.22 | ROS2 包编译构建工具 |

#### 1.2.3 依赖库清单

```
# 核心依赖（ROS2相关）
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-slam-toolbox
ros-humble-ros2-control
ros-humble-xacro
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher
ros-humble-gazebo-ros-pkgs
ros-humble-gazebo-ros-control

# Python运行时依赖
python3-pip
python3-colcon-gtk
python3-vcstool
python3-rosdep

# Python第三方库
flask>=2.3.0
flask-socketio>=5.3.0
eventlet>=0.33.0
scipy>=1.10.0
numpy>=1.24.0

# Gazebo依赖
gazebo11
libgazebo11-dev
```

### 1.3 项目整体运行依赖关系

项目的软件依赖层级结构如下：

```
┌─────────────────────────────────────────────────┐
│           应用层：warehouse_dashboard            │
│           (Flask + SocketIO Web界面)            │
├─────────────────────────────────────────────────┤
│           调度层：fleet_manager                   │
│    (动态调度 · 负荷分析 · 冲突规避 · 任务分配)   │
├─────────────────────────────────────────────────┤
│           导航层：warehouse_navigation           │
│      (Nav2 路径规划 · SLAM Toolbox 建图)        │
├─────────────────────────────────────────────────┤
│           仿真层：warehouse_gazebo               │
│         (Gazebo Classic 11 物理仿真)             │
├─────────────────────────────────────────────────┤
│           描述层：warehouse_description          │
│          (URDF机器人模型 · 世界文件)             │
├─────────────────────────────────────────────────┤
│           消息层：warehouse_msgs                 │
│        (自定义ROS2消息·服务·动作定义)            │
└─────────────────────────────────────────────────┘
                        ↑
               ROS2 Humble (通信中间件)
                        ↑
                   Ubuntu 22.04 (操作系统)
```

各层之间的通信关系：
- `warehouse_msgs`：定义所有包之间传递的消息格式（TaskRequest、RobotStatus、FleetStatus等）
- `warehouse_description`：提供AGV机器人的URDF模型描述和Gazebo世界文件
- `warehouse_gazebo`：在Gazebo中加载世界文件和机器人模型
- `warehouse_navigation`：配置Nav2导航参数和SLAM建图参数
- `fleet_manager`：核心调度层，订阅各机器人状态，发布调度决策
- `warehouse_dashboard`：Web可视化层，通过SocketIO实时展示车队状态

### 1.4 VMware虚拟机环境配置说明

若使用VMware运行Ubuntu 22.04，请按以下步骤配置虚拟机：

**步骤1：创建虚拟机**
- 选择"自定义（高级）"创建类型
- 兼容版本选择"Workstation 17.x"
- 客户机操作系统选择"Linux" → "Ubuntu 64位"
- 分配处理器数量：**4核**，内存：**8192 MB**以上
- 网络类型选择" NAT "或"桥接模式"

**步骤2：开启虚拟化支持**
- 虚拟机设置 → 处理器 → 勾选"虚拟化Intel VT-x/EPT"
- 此选项必须开启，否则Gazebo仿真无法正常运行

**步骤3：安装Ubuntu 22.04**
- 下载Ubuntu 22.04.4 LTS桌面版镜像（ubuntu-22.04.4-desktop-amd64.iso）
- 安装时建议分配至少60GB磁盘空间（动态扩展）
- 安装完成后安装VMware Tools或open-vm-tools-desktop

**步骤4：安装ROS2 Humble**
- 参考本文档第二章进行完整环境搭建

---

## 二、项目环境搭建与依赖配置流程

### 2.1 源码获取与项目目录介绍

**步骤1：创建ROS2工作空间**

```bash
# 创建工作空间目录
mkdir -p ~/multi_robot_warehouse_ws/src
cd ~/multi_robot_warehouse_ws

# 初始化工作空间
source /opt/ros/humble/setup.bash
rosdep init
```

**步骤2：获取项目源码**

将项目源码复制到工作空间的src目录下：

```bash
cd ~/multi_robot_warehouse_ws/src

# 方式一：复制项目文件夹（推荐）
# 将 multi-robot-warehouse-main 文件夹内容复制到此处，
# 并重命名为 multi_robot_warehouse
cp -r /path/to/multi-robot-warehouse-main/* ./

# 方式二：如果是zip压缩包
unzip multi-robot-warehouse-main.zip -d ./
mv multi-robot-warehouse-main/* ./
rm -rf multi-robot-warehouse-main
```

**步骤3：确认项目目录结构**

```bash
cd ~/multi_robot_warehouse_ws
ls -la src/
```

确认src目录下包含以下6个ROS2功能包：

```
src/
├── fleet_manager/              ← 【核心】多AGV动态调度系统
│   ├── config/
│   │   └── fleet_config.yaml  ← 【重要】调度器参数配置
│   ├── fleet_manager/
│   │   ├── fleet_manager_node.py          ← 主调度节点
│   │   ├── adaptive_task_allocator.py     ← 自适应任务分配器
│   │   ├── fleet_load_analyzer.py          ← 负荷分析器
│   │   ├── simple_conflict_resolver.py    ← 简化冲突消解器
│   │   ├── robot_state_manager.py          ← 机器人状态管理
│   │   ├── task_queue.py                  ← 优先级任务队列
│   │   └── warehouse_graph.py             ← 仓储拓扑地图
│   └── launch/
│       └── fleet_manager.launch.py         ← 调度器启动文件
│
├── warehouse_dashboard/        ← Web可视化界面
├── warehouse_description/      ← AGV模型+仿真世界
├── warehouse_gazebo/          ← 仿真启动器
├── warehouse_navigation/       ← Nav2+SLAM配置
└── warehouse_msgs/            ← 自定义ROS2消息定义
```

### 2.2 依赖包逐条安装命令

**阶段1：系统基础依赖**

```bash
# 更新软件包列表
sudo apt update && sudo apt upgrade -y

# 安装基础工具
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    unzip \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-rosinstall \
    python3-rosinstall-generator
```

**阶段2：安装ROS2 Humble（如果尚未安装）**

```bash
# 添加ROS2软件源
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update

# 安装ROS2 Humble桌面版（包含全部组件）
sudo apt install -y ros-humble-desktop

# 安装额外依赖包
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros-control \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-performanceinflation
```

**阶段3：安装Gazebo Classic 11**

```bash
# 添加Gazebo软件源
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo.list'
wget https://packages.osrfoundation.org/gazebo.key -O /tmp/gazebo.key
sudo apt-key add /tmp/gazebo.key
sudo apt update

# 安装Gazebo 11
sudo apt install -y gazebo11 libgazebo11-dev
```

**阶段4：安装Python第三方依赖**

```bash
# 安装Python包管理工具及依赖
pip3 install --upgrade pip setuptools wheel

# 安装Web界面与科学计算依赖
pip3 install \
    flask>=2.3.0 \
    flask-socketio>=5.3.0 \
    eventlet>=0.33.0 \
    scipy>=1.10.0 \
    numpy>=1.24.0

# 安装colcon构建工具（如果尚未安装）
pip3 install colcon-common-extensions
```

**阶段5：安装仿真模型依赖**

```bash
# 安装 turtlebot3 描述文件（部分导航功能依赖）
sudo apt install -y \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-msgs

# 安装 tf 转换工具
sudo apt install -y ros-humble-tf2-tools
```

**阶段6：初始化ROS2环境**

```bash
# 在bashrc中写入ROS2环境变量（永久生效）
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# 创建ROS2工作空间setup脚本
echo 'source ~/multi_robot_warehouse_ws/install/setup.bash' >> ~/.bashrc

# 使环境变量生效
source ~/.bashrc
```

**阶段7：使用rosdep安装剩余依赖**

```bash
cd ~/multi_robot_warehouse_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> **注意**：rosdep安装过程中可能提示某些包找不到，可忽略警告继续。如果某个关键包缺失，手动使用apt安装即可。

### 2.3 环境变量与参数初始化配置

**步骤1：创建ROS2工作空间setup脚本（可选）**

```bash
# 创建方便使用的启动脚本
cat > ~/multi_robot_warehouse_ws/start_env.sh << 'EOF'
#!/bin/bash
# 多AGV仓储机器人仿真环境初始化脚本

echo "======================================"
echo "  多AGV动态调度仿真系统环境初始化"
echo "======================================"

# ROS2环境
source /opt/ros/humble/setup.bash

# Gazebo模型路径（使能找到仿真模型）
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/multi_robot_warehouse_ws/src/warehouse_description/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:~/multi_robot_warehouse_ws/src/warehouse_description/worlds

# TurtleBot3模型路径
export TURTLEBOT3_MODEL=burger

# 使用仿真时间（重要：仿真必须开启）
export ROS_DOMAIN_ID=30

# SLAM参数（可选）
export SVIO_VIDEO_STREAM:=false

echo "[OK] 环境初始化完成"
echo "[OK] 工作空间: ~/multi_robot_warehouse_ws"
echo "[OK] ROS_DOMAIN_ID: 30"

cd ~/multi_robot_warehouse_ws
EOF

chmod +x ~/multi_robot_warehouse_ws/start_env.sh
```

**步骤2：编译整个项目**

```bash
# 进入工作空间
cd ~/multi_robot_warehouse_ws

# 清理旧的编译文件（首次编译可跳过）
rm -rf build/ install/ log/

# 编译所有功能包
colcon build --symlink-install

# 编译成功后，加载编译结果
source install/setup.bash
```

> **编译时间**：首次编译约需5~15分钟（取决于CPU性能）。后续增量编译通常在1~3分钟内完成。

### 2.4 配置文件参数含义详解

#### 2.4.1 调度器核心配置（`fleet_manager/config/fleet_config.yaml`）

```yaml
fleet_manager:
  ros__parameters:

    # ===== 机器人列表 =====
    robot_names:          # 参与调度的AGV车辆名称列表
      - robot_1           # 可在此处添加更多机器人，如 robot_5, robot_6
      - robot_2
      - robot_3
      - robot_4

    # ===== 调度策略配置 =====
    allocation_strategy: "greedy"    # 初始调度策略
                                   # 可选值：
                                   #   "round_robin"    - 轮询分配（低负荷场景）
                                   #   "greedy"         - 贪心分配（中等负荷场景）
                                   #   "hungarian"      - 全局最优分配（高负荷场景）
                                   #   "emergency"      - 紧急模式（高峰期/峰值场景）
                                   #   "load_balanced"  - 负载均衡（多车协同场景）

    auto_adapt_strategy: true       # 【核心功能】开启负荷自适应策略切换
                                   # true: 系统根据实时负荷自动切换最优策略
                                   # false: 固定使用allocation_strategy指定的策略

    fleet_status_hz: 1.0            # 车队状态发布频率（Hz）
                                   # 建议值：0.5~2.0 Hz
                                   # 值越大，Dashboard刷新越快，但CPU占用越高

    # ===== 任务分配权重（多因子评分参数）=====
    weight_distance: 1.0            # 距离因子权重
                                   # 控制机器人到取货点的距离对评分的影响程度
    weight_battery: 2.0            # 电池因子权重
                                   # 值越大，电量低的机器人优先级越低
    weight_load: 0.8                # 工作量因子权重
                                   # 值越大，已完成任务多的机器人越不会被分配新任务
    weight_priority: 3.0            # 任务优先级因子权重
                                   # 值越大，高优先级任务越快被分配
    weight_deadline: 5.0           # 截止时间因子权重
                                   # 值越大，即将达到deadline的任务越优先分配
    battery_reserve: 30.0          # 最低电量阈值（%）
                                   # 电量低于此值的机器人不会被分配新任务

    # ===== 负荷分析参数 =====
    load_analysis_hz: 0.5           # 负荷分析频率（Hz）
                                   # 建议值：0.2~1.0 Hz
                                   # 频率越高，策略切换越灵敏，但计算开销越大

    # ===== 冲突检测参数 =====
    conflict_check_hz: 2.0           # 冲突检测频率（Hz）
    robot_radius: 0.35             # 机器人半径（米）
                                   # 用于碰撞检测的距离计算
    safety_distance: 1.0            # 安全预警距离（米）
                                   # 两车间距小于此值时触发减速避让
    critical_distance: 0.7          # 临界停车距离（米）
                                   # 两车间距小于此值时触发紧急停车

    # ===== 电池与健康管理 =====
    battery_threshold: 20.0         # 低电量阈值（%）
                                   # 电量低于此值时，机器人自动前往充电站
    heartbeat_timeout: 10.0         # 心跳超时时间（秒）
                                   # 超过此时间未收到机器人心跳，标记为ERROR状态
    task_timeout: 120.0            # 任务超时时间（秒）
                                   # 超过此时间任务未完成，标记为失败

    # ===== 机器人物理参数 =====
    loading_duration: 3.0           # 装货时间（秒）
    unloading_duration: 3.0         # 卸货时间（秒）
    avg_robot_speed: 0.5            # 机器人平均移动速度（米/秒）
                                   # 用于估算任务完成时间

    # ===== 仓储地图参数 =====
    warehouse_width: 30.0           # 仓库宽度（米）
    warehouse_height: 20.0          # 仓库高度（米）
    graph_resolution: 0.5          # 路径规划网格分辨率（米/格）
                                   # 值越小，路径越精细，但规划速度越慢

    use_sim_time: true              # 使用仿真时间
                                   # Gazebo仿真时必须设为true
```

#### 2.4.2 导航参数配置（`warehouse_navigation/config/nav2_params.yaml`）

关键导航参数说明：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `controller_server.ranger.min_speed_xy` | 0.0 | 最小线速度 |
| `controller_server.ranger.max_speed_xy` | 0.5 | 最大线速度（m/s） |
| `controller_server.ranger.lookahead_dist` | 0.6 | 前视距离（预测控制） |
| `planner_server.downsample_costmap` | 2 | 降采样倍数 |
| `amcl.update_min_d` | 0.2 | 定位更新最小移动距离 |

#### 2.4.3 机器人spawn参数（`warehouse_gazebo/config/robot_spawn_params.yaml`）

```yaml
robots:
  - name: "robot_1"
    x: -13.0          # 初始X坐标（米）
    y: 7.0            # 初始Y坐标（米）
    z: 0.0            # 初始Z坐标（米）
    yaw: 0.0          # 初始朝向（弧度）

  - name: "robot_2"
    x: 13.0
    y: 7.0
    z: 0.0
    yaw: 3.14159      # 180度朝向

  - name: "robot_3"
    x: -13.0
    y: -7.0
    z: 0.0
    yaw: 0.0

  - name: "robot_4"
    x: 13.0
    y: -7.0
    z: 0.0
    yaw: 3.14159
```

---

## 三、完整项目运行步骤（标准流程）

### 3.1 流程概览

```
┌─────────────────────────────────────────────────────────────┐
│                    完整仿真运行流程                           │
├─────────────────────────────────────────────────────────────┤
│  步骤1：环境初始化 + 启动Gazebo仿真 + 生成AGV模型            │
│      ↓                                                       │
│  步骤2：启动SLAM建图 + 扫描仓储环境 + 保存地图                │
│      ↓                                                       │
│  步骤3：启动Nav2定位 + 启动动态调度系统                       │
│      ↓                                                       │
│  步骤4：发布货物搬运任务 + 监控系统状态                        │
│      ↓                                                       │
│  步骤5：多AGV自动执行任务 + 负荷自适应调度 + 避障纠错          │
│      ↓                                                       │
│  步骤6：通过Dashboard观察实时运行结果                          │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 步骤1：项目启动初始化、加载仓储地图与多AGV车辆模型

**终端1：启动Gazebo仿真环境**

```bash
# 初始化环境
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 启动Gazebo仿真世界（加载仓储地图）
ros2 launch warehouse_gazebo warehouse_world.launch.py use_sim_time:=true
```

> **预期结果**：Gazebo窗口打开，显示仓储环境，包含货架、装货区、卸货区、充电站。

**终端2：生成4台AGV车辆模型**

```bash
# 等待Gazebo完全启动后（约5秒），在新终端中执行
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 生成所有4台AGV机器人
ros2 launch warehouse_gazebo spawn_multi_robot.launch.py
```

> **预期结果**：4台AGV小车出现在Gazebo窗口中，分别位于 (-13, 7)、(13, 7)、(-13, -7)、(13, -7)。

**验证方法**：在Gazebo窗口中可以看到4个差速驱动机器人模型，每个机器人顶部有激光雷达。

### 3.3 步骤2：启动SLAM建图模块、环境扫描建图操作流程

> **说明**：此步骤仅在首次运行或需要重新建图时执行。如果已有保存的地图文件，可直接跳过此步骤，进入步骤3。

**终端3：启动SLAM建图**

```bash
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 选择其中一台机器人（robot_1）启动SLAM
ros2 launch warehouse_navigation slam.launch.py robot_name:=robot_1 use_sim_time:=true
```

> **预期结果**：RViz窗口打开，显示robot_1正在构建地图。激光雷达数据实时显示在地图上。

**终端4：手动操纵机器人完成环境扫描**

```bash
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 安装键盘控制工具（如尚未安装）
sudo apt install -y ros-humble-teleop-twist-keyboard

# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/robot_1/cmd_vel
```

**SLAM建图操作方法**：

```
键盘控制说明：
  i    — 向前移动
  ,    — 向后移动
  j    — 左转
  l    — 右转
  k    — 停止
  space — 紧急停止

操作建议：
  1. 使用"i"键缓慢驾驶robot_1沿仓库边缘行驶一圈
  2. 穿过所有货架之间的通道
  3. 到达每个装货区（x≈-14）和卸货区（x≈14）位置
  4. 确保地图中所有区域都被扫描覆盖
  5. 地图显示完整且无明显噪点后，保存地图
```

**终端5：保存建好的地图**

```bash
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 创建地图保存目录
mkdir -p src/warehouse_navigation/maps

# 保存地图到指定路径
ros2 run nav2_map_server map_saver_cli \
    -f src/warehouse_navigation/maps/warehouse_map
```

> **预期结果**：在 `src/warehouse_navigation/maps/` 目录下生成 `warehouse_map.yaml` 和 `warehouse_map.pgm` 文件。

**关闭SLAM节点**（保存地图后）：

```bash
# 在终端3中按 Ctrl+C 停止SLAM节点
```

### 3.4 步骤3：开启仿真调度系统、发布货物搬运任务

**准备工作：启动Nav2定位**

```bash
# 关闭之前的SLAM节点后，启动Nav2定位
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 为4台机器人分别启动Nav2
ros2 launch warehouse_navigation multi_nav2_bringup.launch.py use_sim_time:=true
```

> **预期结果**：所有4台机器人的Nav2节点启动完成，RViz中显示各机器人的定位状态。

**启动动态调度系统（核心）**

```bash
# 新终端中启动Fleet Manager（动态调度核心）
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 启动调度器，启用自动策略切换
ros2 launch fleet_manager fleet_manager.launch.py \
    use_sim_time:=true \
    allocation_strategy:=greedy \
    auto_adapt_strategy:=true
```

> **预期结果**：终端中输出：
> ```
> [fleet_manager] Fleet Manager 启动 | 机器人数量: 4 | 策略: greedy | 自动适应: True
> [fleet_manager] [策略切换] → 轮询分配（低负荷）
> ```

**启动Web可视化面板**

```bash
# 新终端中启动Dashboard
cd ~/multi_robot_warehouse_ws
source install/setup.bash

ros2 launch warehouse_dashboard dashboard.launch.py
```

> **预期结果**：终端输出 `Dashboard running on http://0.0.0.0:5000`，在浏览器中打开该地址即可看到实时车队状态。

### 3.5 步骤4：多AGV根据任务负荷高低自动动态调度、自主完成货运任务

#### 3.5.1 发布单条货物搬运任务

**方式一：通过ROS2 Topic发布任务**

```bash
# 发布一条标准货物搬运任务
ros2 topic pub --once /task_requests warehouse_msgs/msg/TaskRequest '{
  "task_id": "task_001",
  "task_type": 0,
  "pickup_zone": "loading_dock_1",
  "dropoff_zone": "unloading_dock_1",
  "priority": 2,
  "pickup_location": {
    "position": {"x": -14.0, "y": 3.0, "z": 0.0}
  },
  "dropoff_location": {
    "position": {"x": 14.0, "y": 3.0, "z": 0.0}
  }
}'
```

**方式二：通过Service发布任务**

```bash
ros2 service call /assign_task warehouse_msgs/srv/AssignTask '{
  "task": {
    "task_id": "task_002",
    "task_type": 0,
    "priority": 1,
    "pickup_zone": "loading_dock_2",
    "dropoff_zone": "unloading_dock_2",
    "pickup_location": {
      "position": {"x": -14.0, "y": -3.0, "z": 0.0}
    },
    "dropoff_location": {
      "position": {"x": 14.0, "y": -3.0, "z": 0.0}
    }
  }
}'
```

**方式三：通过Web Dashboard发布任务**

1. 在浏览器中打开 http://localhost:5000
2. 点击"添加任务"按钮
3. 填写取货点、卸货点坐标和优先级
4. 点击"提交"按钮

#### 3.5.2 批量发布任务（模拟峰值场景）

```bash
# 创建一个批量任务发布脚本
cat > ~/task_batch_pub.sh << 'EOF'
#!/bin/bash
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 批量发布10条任务，优先级各异
for i in $(seq 1 10); do
  PRIORITY=$(( (i % 4) ))  # 循环0-3的优先级
  ros2 topic pub --once /task_requests warehouse_msgs/msg/TaskRequest "
  {
    \"task_id\": \"task_batch_$(printf '%03d' $i)\",
    \"task_type\": 0,
    \"pickup_zone\": \"loading_dock_1\",
    \"dropoff_zone\": \"unloading_dock_1\",
    \"priority\": $PRIORITY,
    \"pickup_location\": {
      \"position\": {\"x\": -14.0, \"y\": 3.0, \"z\": 0.0}
    },
    \"dropoff_location\": {
      \"position\": {\"x\": 14.0, \"y\": 3.0, \"z\": 0.0}
    }
  }"
  sleep 0.5  # 每0.5秒发布一条
done
echo "[OK] 批量任务发布完成，共10条"
EOF

chmod +x ~/task_batch_pub.sh
bash ~/task_batch_pub.sh
```

#### 3.5.3 观察动态调度行为

**查看调度日志**：

```bash
# 实时查看Fleet Manager输出
cd ~/multi_robot_warehouse_ws
source install/setup.bash
ros2 run fleet_manager fleet_manager_node
```

**观察关键日志输出**：

```
# 任务到达时输出：
[fleet_manager] [任务到达] task_001 | loading_dock_1 → unloading_dock_1 | 优先级: HIGH | 当前策略: 贪心分配

# 策略自动切换时输出：
[fleet_manager] [策略切换] 轮询分配（低负荷） → 贪心分配（中等负荷） | 负荷: MODERATE | pending=5 | available=4 | utilization=0.50

# 任务分配时输出：
[fleet_manager] [自动分配] task_001 → robot_1 | loading_dock_1 → unloading_dock_1 | 策略: 贪心分配（中等负荷）

# 任务完成时输出：
[fleet_manager] [任务完成] task_001 | robot_1 | 耗时 52.3s | 策略: 贪心分配（中等负荷）

# 负荷高峰时策略切换：
[fleet_manager] [策略切换] 贪心分配（中等负荷） → 全局最优（高负荷） | 负荷: HIGH | pending=12 | available=3 | utilization=0.75
```

**查看车队状态话题**：

```bash
# 查看实时车队状态
ros2 topic echo /fleet_status --once

# 持续监听（每1秒刷新）
watch -n 1 "ros2 topic echo /fleet_status --once"
```

**查看调度策略切换历史**：

```bash
# 查看所有机器人的实时状态
ros2 topic echo /fleet_status
```

### 3.6 步骤5：意外障碍物触发简易规避与纠错逻辑运行说明

> **说明**：本项目的冲突消解采用简化策略，不再使用复杂的CBS约束树搜索，仅保留运行时检测与简单纠错行为。

#### 3.6.1 正常运行中的冲突检测

系统每0.5秒自动检测所有机器人之间的间距：

```
[fleet_manager] [冲突检测] 检测中 | robot_1: (-5.2, 3.1) | robot_2: (-5.1, 3.3) | 间距: 0.25m
```

#### 3.6.2 触发预警避让（间距 < 1.0m）

当两车间距进入预警范围时，低优先级车辆自动减速：

```
[fleet_manager] [冲突预警] robot_3 减速让行 robot_1
```

#### 3.6.3 触发紧急停车（间距 < 0.7m）

当两车间距进入临界范围时，立即触发停车：

```
[fleet_manager] [冲突避让] robot_3 停止等待 robot_1
```

#### 3.6.4 触发死锁检测与重规划

当机器人陷入停滞超过阈值时：

```
[fleet_manager] [死锁检测] robot_2 陷入停滞，触发重新规划
[fleet_manager] [死锁解决] robot_2 重规划完成，恢复执行
```

#### 3.6.5 手动测试冲突场景

```bash
# 方法1：同时发布两个目标点相近的任务
# 创建两条目标点非常接近的任务
ros2 topic pub --once /task_requests warehouse_msgs/msg/TaskRequest '
{
  "task_id": "conflict_test_1",
  "task_type": 0,
  "pickup_zone": "loading_dock_1",
  "dropoff_zone": "unloading_dock_1",
  "priority": 1,
  "pickup_location": {"position": {"x": -14.0, "y": 3.0, "z": 0.0}},
  "dropoff_location": {"position": {"x": 14.0, "y": 3.0, "z": 0.0}}
}'

ros2 topic pub --once /task_requests warehouse_msgs/msg/TaskRequest '
{
  "task_id": "conflict_test_2",
  "task_type": 0,
  "pickup_zone": "loading_dock_1",
  "dropoff_zone": "unloading_dock_1",
  "priority": 1,
  "pickup_location": {"position": {"x": -14.0, "y": 3.0, "z": 0.0}},
  "dropoff_location": {"position": {"x": 14.0, "y": 3.0, "z": 0.0}}
}'
```

> **预期**：两车从不同位置向同一取货点行驶时，系统检测到路径交汇，触发避让逻辑。

---

## 四、不同实验场景的运行配置说明

### 4.1 低负荷、常规任务场景配置与运行方式

**场景特征**：
- 任务稀疏（pending_tasks < available_robots × 0.5）
- 机器人利用率低（< 50%）
- 车辆大部分处于空闲状态

**配置方法**：

```bash
# 启动调度器，指定轮询策略（低负荷最优）
ros2 launch fleet_manager fleet_manager.launch.py \
    use_sim_time:=true \
    allocation_strategy:=round_robin \
    auto_adapt_strategy:=true
```

**验证方法**：

```bash
# 发布少量任务
ros2 topic pub --once /task_requests warehouse_msgs/msg/TaskRequest '
{
  "task_id": "low_load_001",
  "task_type": 0,
  "priority": 1,
  "pickup_zone": "loading_dock_1",
  "dropoff_zone": "unloading_dock_1",
  "pickup_location": {"position": {"x": -14.0, "y": 3.0, "z": 0.0}},
  "dropoff_location": {"position": {"x": 14.0, "y": 3.0, "z": 0.0}}
}'
```

**观察要点**：
- 策略标签应为"轮询分配（低负荷）"
- 各机器人交替接收任务，任务分配均匀
- 机器人利用率保持在50%以下

### 4.2 高负荷、任务峰值密集场景配置与运行方式

**场景特征**：
- 任务密集（pending_tasks > available_robots × 2.0）
- 机器人利用率高（> 75%）
- 调度系统承受高负载

**配置方法**：

```bash
# 启动调度器，指定匈牙利算法（高负荷最优）
ros2 launch fleet_manager fleet_manager.launch.py \
    use_sim_time:=true \
    allocation_strategy:=hungarian \
    auto_adapt_strategy:=true
```

**批量任务压测脚本**：

```bash
cat > ~/peak_load_test.sh << 'EOF'
#!/bin/bash
cd ~/multi_robot_warehouse_ws
source install/setup.bash

echo "===== 开始高负荷压测 ====="

# 同时发布20条任务，模拟峰值
for i in $(seq 1 20); do
  PRIORITY=$(( (i % 4) ))
  # 随机选择装/卸货区
  if [ $((i % 2)) -eq 0 ]; then
    PICKUP_X=-14.0; PICKUP_Y=3.0; DROPOFF_X=14.0; DROPOFF_Y=3.0
  else
    PICKUP_X=-14.0; PICKUP_Y=-3.0; DROPOFF_X=14.0; DROPOFF_Y=-3.0
  fi

  ros2 topic pub --once /task_requests warehouse_msgs/msg/TaskRequest "
  {
    \"task_id\": \"peak_$(printf '%03d' $i)\",
    \"task_type\": 0,
    \"pickup_zone\": \"loading_dock_1\",
    \"dropoff_zone\": \"unloading_dock_1\",
    \"priority\": $PRIORITY,
    \"pickup_location\": {\"position\": {\"x\": $PICKUP_X, \"y\": $PICKUP_Y, \"z\": 0.0}},
    \"dropoff_location\": {\"position\": {\"x\": $DROPOFF_X, \"y\": $DROPOFF_Y, \"z\": 0.0}}
  }"
  sleep 0.2
done

echo "[OK] 20条高优先级任务已发布"
echo "预期：系统自动切换到 全局最优（高负荷） 策略"
EOF

chmod +x ~/peak_load_test.sh
bash ~/peak_load_test.sh
```

**观察要点**：
- 日志中应出现 `[策略切换]` 消息，显示切换到"全局最优（高负荷）"
- `pending_task_count` 快速增长，`available_robots` 可能降为0
- 系统优先保证高优先级任务完成

### 4.3 多车辆、复杂仓库场景实验配置方法

#### 4.3.1 扩展机器人数量（扩展到6台）

**步骤1：修改配置文件**

编辑 `fleet_manager/config/fleet_config.yaml`：

```yaml
robot_names:
  - robot_1
  - robot_2
  - robot_3
  - robot_4
  - robot_5    # 新增
  - robot_6    # 新增
```

**步骤2：修改spawn参数**

编辑 `warehouse_gazebo/config/robot_spawn_params.yaml`：

```yaml
robots:
  - name: "robot_1"
    x: -13.0; y: 8.0; z: 0.0; yaw: 0.0
  - name: "robot_2"
    x: 13.0; y: 8.0; z: 0.0; yaw: 3.14159
  - name: "robot_3"
    x: -13.0; y: 0.0; z: 0.0; yaw: 0.0
  - name: "robot_4"
    x: 13.0; y: 0.0; z: 0.0; yaw: 3.14159
  - name: "robot_5"
    x: -13.0; y: -8.0; z: 0.0; yaw: 0.0
  - name: "robot_6"
    x: 13.0; y: -8.0; z: 0.0; yaw: 3.14159
```

**步骤3：修改URDF（新增机器人的命名空间）**

```bash
# 为robot_5和robot_6生成模型（参考现有模型）
cp src/warehouse_description/urdf/agv_robot.urdf.xacro \
   src/warehouse_description/urdf/agv_robot_5.urdf.xacro
# 编辑命名空间为 robot_5
```

**步骤4：修改spawn launch文件**

编辑 `warehouse_gazebo/launch/spawn_multi_robot.launch.py`，添加 robot_5 和 robot_6 的spawn逻辑。

**步骤5：重新编译并运行**

```bash
cd ~/multi_robot_warehouse_ws
colcon build --symlink-install
source install/setup.bash
```

#### 4.3.2 增大仓库规模

修改 `warehouse_graph.py` 中的 `WarehouseGraph.__init__()` 参数：

```python
# 增大仓库尺寸
self.__init__(width=50.0, height=35.0, resolution=0.5)
```

同时修改 `warehouse_description/worlds/warehouse.world` 中的物理世界尺寸。

---

## 五、项目启动常见问题与报错解决方案

### 5.1 依赖缺失类问题

#### 问题A：`ModuleNotFoundError: No module named 'flask'`

**原因**：Python Flask依赖未安装。

**解决方法**：

```bash
pip3 install flask flask-socketio eventlet scipy numpy
```

#### 问题B：`package 'warehouse_msgs' not found`

**原因**：ROS2包未正确编译，或环境变量未加载。

**解决方法**：

```bash
# 重新编译并source
cd ~/multi_robot_warehouse_ws
colcon build --symlink-install
source install/setup.bash

# 验证包是否可见
ros2 pkg list | grep warehouse
```

#### 问题C：`Couldn't find an AF_INET address for [...]`

**原因**：ROS2节点之间无法发现对方，网络配置问题。

**解决方法**：

```bash
# 设置ROS_DOMAIN_ID（所有终端保持一致）
export ROS_DOMAIN_ID=30

# 如果使用虚拟机，检查网络模式是否为NAT或桥接
# 建议在所有终端的~/.bashrc中添加：
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
```

#### 问题D：`colcon: command not found`

**原因**：colcon工具未安装。

**解决方法**：

```bash
pip3 install colcon-common-extensions
```

### 5.2 地图加载失败类问题

#### 问题E：`Failed to load map: file not found`

**原因**：地图文件路径不正确，或地图尚未保存。

**解决方法**：

```bash
# 检查地图文件是否存在
ls -la ~/multi_robot_warehouse_ws/src/warehouse_navigation/maps/

# 如果地图文件不存在，执行SLAM建图并保存（参见本文档3.3节）

# 检查nav2_params.yaml中的地图路径配置
cat ~/multi_robot_warehouse_ws/src/warehouse_navigation/config/nav2_params.yaml | grep -A2 "map_server"
```

#### 问题F：SLAM建图时地图出现大量噪点

**原因**：激光雷达数据质量差，或机器人运动过快。

**解决方法**：

```bash
# 降低机器人移动速度
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/robot_1/cmd_vel

# 在RViz中降低LaserScan的显示阈值
# 将"Size (m)"从默认0.05降低到0.02

# 重新扫描地图，缓慢行驶
```

### 5.3 小车初始化异常类问题

#### 问题G：Gazebo中机器人spawn后坠落/翻倒

**原因**：URDF模型物理参数（质量、惯性）配置错误，或Gazebo重力设置异常。

**解决方法**：

```bash
# 在启动Gazebo时确保重力设置正确
ros2 launch warehouse_gazebo warehouse_world.launch.py \
    use_sim_time:=true \
    set_physics_params:="--set physics timestep 0.001"
```

检查 `agv_robot.urdf.xacro` 中的惯性参数：

```xml
<!-- 确保包含正确的惯性矩阵 -->
<inertial>
  <mass value="10.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

#### 问题H：机器人odom漂移严重

**原因**：差速驱动参数配置不当，或Gazebo中轮子打滑。

**解决方法**：

```bash
# 检查并调整差速驱动参数
# 编辑 agv_robot.urdf.xacro 中的 wheel_separation 和 wheel_radius

# 推荐参数
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <ros>
    <namespace>/robot_1</namespace>
  </ros>
  <update_rate>50</update_rate>
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.45</wheel_separation>
  <wheel_radius>0.08</wheel_radius>
  <max_wheel_torque>20.0</max_wheel_torque>
  <max_linear_velocity>0.5</max_linear_velocity>
</plugin>
```

### 5.4 调度任务卡死类问题

#### 问题I：任务发布后机器人不移动

**原因**：Nav2 action server未就绪，或路径规划失败。

**解决方法**：

```bash
# 检查Nav2服务是否正常运行
ros2 service list | grep navigate_to_pose

# 检查是否有路径规划错误
ros2 topic echo /robot_1/feedback 2>/dev/null || echo "无反馈数据"

# 重启Nav2
# 关闭当前Nav2终端，重新运行：
ros2 launch warehouse_navigation multi_nav2_bringup.launch.py use_sim_time:=true
```

#### 问题J：任务一直处于PENDING状态，无机器人接收

**原因**：所有机器人都不满足分配条件（电量不足、处于ERROR状态等）。

**解决方法**：

```bash
# 查看所有机器人状态
ros2 service call /get_robot_status warehouse_msgs/srv/GetRobotStatus '{}'

# 检查是否有机器人处于ERROR状态
# 查看Fleet Manager日志中的心跳超时警告

# 如果是电量问题，等待自动充电完成，或手动降低battery_threshold参数
```

#### 问题K：机器人卡在货架之间不动

**原因**：全局路径规划失败，或局部路径被障碍物阻挡。

**解决方法**：

```bash
# 重启该机器人的导航
# 在RViz中手动点击"2D Pose Estimate"重置机器人位姿
# 然后重新发送目标点

# 如果频繁发生，检查warehouse.world中的货架碰撞参数
# 增大货架与通道之间的间隙
```

### 5.5 仿真卡顿与性能问题

#### 问题L：Gazebo运行极慢（低于实时率）

**原因**：虚拟机资源不足，或Gazebo渲染设置过高。

**解决方法**：

```bash
# 方法1：关闭Gazebo渲染窗口（无头模式）
ros2 launch warehouse_gazebo warehouse_world.launch.py \
    use_sim_time:=true \
    headless:=true

# 方法2：降低Gazebo物理更新率
# 编辑 warehouse.world 文件
# 将 <max_step_size>0.001</max_step_size> 改为 <max_step_size>0.005</max_step_size>
# 将 <real_time_update_rate>1000</real_time_update_rate> 改为 <real_time_update_rate>100</real_time_update_rate>

# 方法3：减少仿真机器人数量（临时测试）
# 启动时只spawn 2台机器人
ros2 launch warehouse_gazebo spawn_single_robot.launch.py robot_name:=robot_1
```

#### 问题M：ROS2通信延迟高，任务分配不及时

**原因**：多节点竞争CPU资源，或callback group配置不当。

**解决方法**：

```bash
# 确保Fleet Manager使用多线程executor
# 在启动命令中添加线程数参数
# 注意：代码中已设置为8线程executor，一般无需修改

# 检查系统负载
top -bn1 | head -20

# 降低各节点的发布频率
# 在fleet_config.yaml中降低 fleet_status_hz 和 conflict_check_hz
```

### 5.6 Web Dashboard无法访问

#### 问题N：Dashboard启动成功但浏览器无法打开

**解决方法**：

```bash
# 检查Dashboard是否真的在运行
ps aux | grep dashboard

# 检查端口是否被占用
sudo lsof -i :5000

# 查看Dashboard日志中的错误
ros2 launch warehouse_dashboard dashboard.launch.py
```

#### 问题O：Dashboard显示数据不更新

**原因**：ROS2与Dashboard之间的SocketIO桥接中断。

**解决方法**：

```bash
# 重启Dashboard
# 按Ctrl+C停止后重新启动
ros2 launch warehouse_dashboard dashboard.launch.py

# 检查fleet_status话题是否正常发布
ros2 topic echo /fleet_status --once
```

---

## 附录：快速启动命令汇总

```bash
# ===== 一键启动完整仿真环境 =====
cd ~/multi_robot_warehouse_ws
source install/setup.bash

# 终端1：Gazebo仿真
ros2 launch warehouse_gazebo full_simulation.launch.py use_sim_time:=true

# 等待Gazebo启动完成（约5秒）

# 终端2：Nav2导航（4台机器人）
ros2 launch warehouse_navigation multi_nav2_bringup.launch.py use_sim_time:=true

# 终端3：动态调度系统
ros2 launch fleet_manager fleet_manager.launch.py \
    use_sim_time:=true \
    allocation_strategy:=greedy \
    auto_adapt_strategy:=true

# 终端4：Web可视化
ros2 launch warehouse_dashboard dashboard.launch.py

# 终端5：发布测试任务
ros2 topic pub --once /task_requests warehouse_msgs/msg/TaskRequest '
{
  "task_id": "demo_001",
  "task_type": 0,
  "priority": 2,
  "pickup_zone": "loading_dock_1",
  "dropoff_zone": "unloading_dock_1",
  "pickup_location": {"position": {"x": -14.0, "y": 3.0, "z": 0.0}},
  "dropoff_location": {"position": {"x": 14.0, "y": 3.0, "z": 0.0}}
}'

# 打开浏览器访问：http://localhost:5000

