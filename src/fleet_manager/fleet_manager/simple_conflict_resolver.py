"""
Simple Conflict Avoidance — 简化冲突消解器

完全重写，替代原有的 CBS（Conflict-Based Search）复杂算法。

设计原则（老师要求）：
- 弱化局部路径规划，不重点研究动态避障算法
- 仅保留"遇到意外障碍物时的简单规避/纠错行为"

核心机制：
1. 运行时冲突检测：实时检测两车间距离是否过近
2. 主动避让策略：检测到冲突时，低优先级机器人主动减速/绕行
3. 死锁检测：识别长期停滞的机器人并触发重规划
4. 交汇点管理：管理狭窄通道的通行权，防止对向死锁

避让优先级规则：
- 任务截止时间更紧迫的机器人优先通行
- 已进入交汇区的机器人优先通行
- 等待时间更长的机器人优先通行
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Set, Tuple

from .warehouse_graph import Node, WarehouseGraph


RobotId = str


class AvoidanceAction(IntEnum):
    """机器人应采取的规避动作。"""
    NONE     = 0
    SLOWDOWN = 1   # 减速让行
    STOP     = 2   # 停止等待
    DETOUR   = 3   # 绕行（换条路径）


@dataclass
class ConflictEvent:
    """检测到的冲突事件。"""
    robot_1:    RobotId
    robot_2:    RobotId
    distance:   float
    detected_at: float
    resolved:   bool = False


@dataclass
class AvoidanceDecision:
    """针对单个机器人的规避决策。"""
    robot_id:   RobotId
    action:     AvoidanceAction
    target_robot: Optional[RobotId]
    priority:   float


@dataclass
class NarrowPassage:
    """狭窄通道管理（货架之间的通道）。"""
    name:       str
    node_ids:   Set[Tuple[float, float]]
    reserved_by: Optional[RobotId] = None
    reserved_at: float = 0.0
    timeout:    float = 30.0


class SimpleConflictResolver:
    """
    简化冲突消解器。

    功能：
    - 实时距离检测与主动避让
    - 死锁检测与重规划触发
    - 狭窄通道令牌管理
    - 碰撞预警（提前减速）

    不包含：完整CBS约束树、时空调度、路径重规划
    """

    # 物理参数
    ROBOT_RADIUS:       float = 0.35   # 米
    SAFETY_DISTANCE:    float = 1.0    # 预警距离（米）
    CRITICAL_DISTANCE:  float = 0.7    # 临界距离（米）

    # 时序参数
    DEADLOCK_WINDOW:    int = 20       # 死锁检测窗口（采样数）
    DEADLOCK_THRESHOLD: float = 0.15   # 最小移动量阈值（米）

    def __init__(self, graph: WarehouseGraph) -> None:
        self.graph = graph
        self.conflict_count = 0
        self.avoidance_count = 0
        self.deadlock_count = 0

        # 运行时状态
        self._active_conflicts: Dict[Tuple[RobotId, RobotId], ConflictEvent] = {}
        self._narrow_passages: Dict[str, NarrowPassage] = {}
        self._yield_records: Dict[RobotId, float] = {}  # robot_id -> 最后一次让行时间
        self._stuck_robots: Set[RobotId] = set()
        self._lock = __import__("threading").Lock()

        self._init_narrow_passages()

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------

    def _init_narrow_passages(self) -> None:
        """初始化仓库中的狭窄通道。"""
        passages = [
            NarrowPassage(
                name="corridor_main_h",
                node_ids={(float(x), float(y))
                          for x in [-15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0]
                          for y in [-2.0, 2.0]},
            ),
            NarrowPassage(
                name="corridor_main_v",
                node_ids={(float(x), float(y))
                          for x in [-2.0, 2.0]
                          for y in [-10.0, -5.0, 0.0, 5.0, 10.0]},
            ),
        ]
        for p in passages:
            self._narrow_passages[p.name] = p

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def check_conflicts(
        self,
        robot_positions: Dict[RobotId, Tuple[float, float]],
        robot_states: Optional[Dict[RobotId, str]] = None,
        robot_deadlines: Optional[Dict[RobotId, Optional[float]]] = None,
    ) -> Dict[RobotId, AvoidanceDecision]:
        """
        主入口：检测所有机器人之间的冲突，返回每个机器人应采取的规避决策。

        Args:
            robot_positions: {robot_id: (x, y)}
            robot_states: 可选，{robot_id: state_name}
            robot_deadlines: 可选，{robot_id: deadline_timestamp}

        Returns:
            {robot_id: AvoidanceDecision}
        """
        decisions: Dict[RobotId, AvoidanceDecision] = {}
        robot_ids = list(robot_positions.keys())
        now = time.time()

        # 1. 检测距离冲突
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                r1, r2 = robot_ids[i], robot_ids[j]
                x1, y1 = robot_positions[r1]
                x2, y2 = robot_positions[r2]
                dist = math.hypot(x2 - x1, y2 - y1)

                key = tuple(sorted([r1, r2]))
                if dist < self.SAFETY_DISTANCE:
                    # 记录活跃冲突
                    with self._lock:
                        self._active_conflicts[key] = ConflictEvent(
                            robot_1=r1, robot_2=r2,
                            distance=dist, detected_at=now,
                        )
                    self.conflict_count += 1

                    # 决定谁让谁
                    action1, action2 = self._decide_yield(
                        r1, r2, dist, robot_deadlines
                    )
                    decisions[r1] = action1
                    decisions[r2] = action2

                    if action1.action != AvoidanceAction.NONE:
                        self.avoidance_count += 1
                    if action2.action != AvoidanceAction.NONE:
                        self.avoidance_count += 1
                else:
                    # 冲突解除
                    with self._lock:
                        self._active_conflicts.pop(key, None)

        return decisions

    def detect_deadlock(
        self,
        position_history: Dict[RobotId, List[Tuple[float, float]]],
        window: int = DEADLOCK_WINDOW,
        threshold: float = DEADLOCK_THRESHOLD,
    ) -> List[RobotId]:
        """
        检测陷入死锁的机器人（长期停滞不前）。

        Returns: 死锁机器人 ID 列表
        """
        deadlocked: List[RobotId] = []
        for robot_id, history in position_history.items():
            if len(history) < window:
                continue
            recent = history[-window:]
            total_movement = sum(
                math.hypot(
                    recent[i][0] - recent[i - 1][0],
                    recent[i][1] - recent[i - 1][1],
                )
                for i in range(1, len(recent))
            )
            if total_movement < threshold:
                deadlocked.append(robot_id)
                self.deadlock_count += 1
        return deadlocked

    def resolve_deadlock(
        self,
        robot_id: str,
        current_goal: Tuple[float, float],
        positions: Dict[RobotId, Tuple[float, float]],
    ) -> Optional[Tuple[float, float]]:
        """
        死锁时返回替代目标点（绕行）。

        策略：尝试将目标偏移到当前停车点的旁侧。
        """
        if robot_id not in positions:
            return None
        x, y = positions[robot_id]

        # 尝试 4 个方向的偏移
        offsets = [(1.5, 0.0), (-1.5, 0.0), (0.0, 1.5), (0.0, -1.5)]
        import random
        random.seed(hash(robot_id))
        random.shuffle(offsets)

        for dx, dy in offsets:
            alt_x = x + dx
            alt_y = y + dy
            # 确认替代点在可通行区域
            node = self.graph.world_to_node(alt_x, alt_y)
            if node is not None:
                return (float(alt_x), float(alt_y))

        return None

    def check_narrow_passage(
        self,
        robot_id: str,
        position: Tuple[float, float],
    ) -> Optional[str]:
        """
        检查机器人是否正在进入/穿越狭窄通道。

        Returns: 通道名称或 None
        """
        px, py = position
        for name, passage in self._narrow_passages.items():
            for nx, ny in passage.node_ids:
                if math.hypot(px - nx, py - ny) < self.SAFETY_DISTANCE:
                    return name
        return None

    def request_passage(
        self,
        robot_id: str,
        passage_name: str,
        deadline: Optional[float] = None,
    ) -> bool:
        """
        申请通过狭窄通道。

        规则：先到先得，已获通行权的机器人 timeout 后自动释放。
        """
        passage = self._narrow_passages.get(passage_name)
        if passage is None:
            return False
        now = time.time()

        # 超时自动释放
        if passage.reserved_by and (now - passage.reserved_at) > passage.timeout:
            passage.reserved_by = None

        # 已有通行权
        if passage.reserved_by == robot_id:
            passage.reserved_at = now
            return True

        # 通道空闲
        if passage.reserved_by is None:
            passage.reserved_by = robot_id
            passage.reserved_at = now
            return True

        return False

    def release_passage(self, robot_id: str, passage_name: str) -> None:
        """释放狭窄通道通行权。"""
        passage = self._narrow_passages.get(passage_name)
        if passage and passage.reserved_by == robot_id:
            passage.reserved_by = None

    def get_active_conflicts(self) -> List[ConflictEvent]:
        return list(self._active_conflicts.values())

    def get_metrics(self) -> Dict:
        return {
            "total_conflicts": self.conflict_count,
            "total_avoidances": self.avoidance_count,
            "total_deadlocks": self.deadlock_count,
            "active_conflicts": len(self._active_conflicts),
        }

    def reset_metrics(self) -> None:
        self.conflict_count = 0
        self.avoidance_count = 0
        self.deadlock_count = 0

    # ------------------------------------------------------------------
    # 内部决策逻辑
    # ------------------------------------------------------------------

    def _decide_yield(
        self,
        r1: RobotId,
        r2: RobotId,
        dist: float,
        deadlines: Optional[Dict[RobotId, Optional[float]]] = None,
    ) -> Tuple[AvoidanceDecision, AvoidanceDecision]:
        """
        决定两辆冲突机器人中谁让谁。

        优先级规则（按优先级从高到低）：
        1. 截止时间更紧迫的机器人优先通行
        2. 已在交汇区的机器人优先通行
        3. 等待时间更长的机器人优先通行
        4. 随机（作为最终兜底）
        """
        deadline1 = deadlines.get(r1) if deadlines else None
        deadline2 = deadlines.get(r2) if deadlines else None
        now = time.time()

        # 截止时间判断
        has_dl1 = deadline1 is not None
        has_dl2 = deadline2 is not None

        urgency1 = 0.0
        urgency2 = 0.0
        if has_dl1:
            remaining = deadline1 - now
            if remaining <= 0:
                urgency1 = 100.0
            else:
                urgency1 = max(0.0, 10.0 / remaining)
        if has_dl2:
            remaining = deadline2 - now
            if remaining <= 0:
                urgency2 = 100.0
            else:
                urgency2 = max(0.0, 10.0 / remaining)

        # 等待时间判断
        wait1 = self._yield_records.get(r1, 0.0)
        wait2 = self._yield_records.get(r2, 0.0)

        # 综合优先级
        priority1 = urgency1 + wait1 * 0.5
        priority2 = urgency2 + wait2 * 0.5

        if dist < self.CRITICAL_DISTANCE:
            # 临界距离：必须立即停车
            if priority1 >= priority2:
                return (
                    AvoidanceDecision(r1, AvoidanceAction.NONE, None, priority1),
                    AvoidanceDecision(r2, AvoidanceAction.STOP, r1, priority2),
                )
            else:
                return (
                    AvoidanceDecision(r1, AvoidanceAction.STOP, r2, priority1),
                    AvoidanceDecision(r2, AvoidanceAction.NONE, None, priority2),
                )
        else:
            # 预警距离：减速让行
            if priority1 >= priority2:
                self._yield_records[r2] = now
                return (
                    AvoidanceDecision(r1, AvoidanceAction.NONE, None, priority1),
                    AvoidanceDecision(r2, AvoidanceAction.SLOWDOWN, r1, priority2),
                )
            else:
                self._yield_records[r1] = now
                return (
                    AvoidanceDecision(r1, AvoidanceAction.SLOWDOWN, r2, priority1),
                    AvoidanceDecision(r2, AvoidanceAction.NONE, None, priority2),
                )
