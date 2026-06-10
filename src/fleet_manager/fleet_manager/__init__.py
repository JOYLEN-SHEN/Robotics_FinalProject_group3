"""
Fleet Manager Package — 多AGV动态调度系统

核心模块：
- fleet_manager_node:      ROS2主节点，动态调度核心
- adaptive_task_allocator: 自适应任务分配器（5种策略）
- fleet_load_analyzer:     负荷分析器（负荷等级 + 策略推荐）
- simple_conflict_resolver: 简化冲突消解器（替代CBS）
- robot_state_manager:     机器人状态管理
- task_queue:              优先级任务队列
- warehouse_graph:         仓储拓扑地图
"""

from .robot_state_manager import RobotStateManager, RobotState, RobotInfo
from .task_queue import TaskQueue, WarehouseTask, TaskPriority, TaskState
from .adaptive_task_allocator import AdaptiveTaskAllocator, AllocationStrategy
from .fleet_load_analyzer import FleetLoadAnalyzer, LoadLevel, LoadMetrics
from .simple_conflict_resolver import SimpleConflictResolver, AvoidanceAction
from .warehouse_graph import WarehouseGraph, Node, Zone

__all__ = [
    "FleetManagerNode",
    "RobotStateManager",
    "RobotState",
    "RobotInfo",
    "TaskQueue",
    "WarehouseTask",
    "TaskPriority",
    "TaskState",
    "AdaptiveTaskAllocator",
    "AllocationStrategy",
    "FleetLoadAnalyzer",
    "LoadLevel",
    "LoadMetrics",
    "SimpleConflictResolver",
    "AvoidanceAction",
    "WarehouseGraph",
    "Node",
    "Zone",
]
