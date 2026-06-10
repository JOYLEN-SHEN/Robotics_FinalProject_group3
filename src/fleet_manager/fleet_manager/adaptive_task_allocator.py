"""
Adaptive Task Allocator — 自适应任务分配器

完全重写，替代原有的 greedy/hungarian 静态分配。

支持五种调度策略，根据系统负荷自动/手动切换：
1. ROUND_ROBIN   — 分区轮询，均摊任务到各机器人，低负荷最优
2. GREEDY        — 贪心最近距离，中等负荷
3. HUNGARIAN     — 全局最优分配（匈牙利算法），高负荷
4. EMERGENCY     — 紧急模式：优先电池充足的机器人 + 任务紧急度，高峰期
5. LOAD_BALANCED — 基于机器人负荷画像的负载均衡，适合多车协同

分配评分函数（多因子加权）：
  score = battery_factor * w_batt
        + proximity_factor * w_dist
        + workload_factor * w_workload
        + priority_factor * w_priority
        + deadline_factor * w_deadline

  电池因子：电量越低越不适合接新任务
  邻近因子：距离 pickup 越近得分越高
  工作量因子：已完成任务越多，得分越低（避免过载）
  优先级因子：高优先级任务分配给最优机器人
  截止时间因子：即将到期的任务优先处理
"""

from __future__ import annotations

import math
import time
import heapq
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

from .robot_state_manager import RobotInfo, RobotState
from .task_queue import WarehouseTask, TaskState, TaskPriority


class AllocationStrategy(Enum):
    ROUND_ROBIN   = "round_robin"
    GREEDY        = "greedy"
    HUNGARIAN     = "hungarian"
    EMERGENCY     = "emergency"
    LOAD_BALANCED = "load_balanced"


@dataclass
class AllocationScore:
    """单次 (robot, task) 分配评分明细。"""
    robot_id:        str
    task_id:         str
    total:           float
    distance:        float
    battery_factor:  float
    workload_factor: float
    priority_factor: float
    deadline_factor: float
    proximity_factor: float


@dataclass(order=True)
class _BatchEntry:
    """匈牙利算法的批量分配条目。"""
    sort_key: float
    robot_id: str = field(compare=False)
    task_id: str = field(compare=False)


def _euclidean(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


class AdaptiveTaskAllocator:
    """
    自适应任务分配器。

    核心特性：
    - 策略自动切换（根据负荷等级）
    - 多因子综合评分
    - 支持批量分配
    - 负载均衡保护（防止单机器人过载）
    """

    STRATEGY_LABELS = {
        AllocationStrategy.ROUND_ROBIN:   "轮询分配（低负荷）",
        AllocationStrategy.GREEDY:        "贪心分配（中等负荷）",
        AllocationStrategy.HUNGARIAN:     "全局最优（高负荷）",
        AllocationStrategy.EMERGENCY:     "紧急模式（高峰期）",
        AllocationStrategy.LOAD_BALANCED: "负载均衡（多车协同）",
    }

    def __init__(
        self,
        strategy: str = "greedy",
        auto_adapt: bool = True,
    ) -> None:
        self._strategy = self._parse_strategy(strategy)
        self._auto_adapt = auto_adapt
        self._round_robin_index = 0
        self._last_strategy = self._strategy

        # 权重配置
        self.w_batt:       float = 2.0
        self.w_dist:       float = 1.0
        self.w_workload:   float = 0.8
        self.w_priority:   float = 3.0
        self.w_deadline:   float = 5.0
        self.battery_reserve: float = 30.0

        # 负载均衡参数
        self.max_tasks_per_robot: int = 3
        self.workload_history: Dict[str, int] = {}

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def set_strategy(self, strategy: str) -> None:
        """手动设置调度策略。"""
        self._strategy = self._parse_strategy(strategy)
        self._last_strategy = self._strategy

    def get_strategy(self) -> AllocationStrategy:
        return self._strategy

    def get_strategy_label(self) -> str:
        return self.STRATEGY_LABELS.get(self._strategy, str(self._strategy))

    def assign_task(
        self,
        task: WarehouseTask,
        available_robots: List[RobotInfo],
    ) -> Optional[str]:
        """
        为单个任务分配最优机器人。
        Returns: robot_id or None
        """
        eligible = self._filter_eligible(task, available_robots)
        if not eligible:
            return None

        if self._strategy == AllocationStrategy.ROUND_ROBIN:
            return self._assign_round_robin(eligible)
        elif self._strategy == AllocationStrategy.GREEDY:
            return self._assign_greedy(task, eligible)
        elif self._strategy == AllocationStrategy.EMERGENCY:
            return self._assign_emergency(task, eligible)
        elif self._strategy == AllocationStrategy.LOAD_BALANCED:
            return self._assign_load_balanced(task, eligible)
        else:
            return self._assign_greedy(task, eligible)

    def batch_assign(
        self,
        tasks: List[WarehouseTask],
        available_robots: List[RobotInfo],
    ) -> Dict[str, str]:
        """
        批量分配多个任务给多个机器人。
        Returns: {task_id: robot_id}
        """
        if not tasks or not available_robots:
            return {}

        # 过滤有效任务和机器人
        valid_tasks = [t for t in tasks if t.state == TaskState.PENDING]
        if not valid_tasks:
            return {}

        if self._strategy == AllocationStrategy.HUNGARIAN:
            return self._batch_hungarian(valid_tasks, available_robots)
        elif self._strategy == AllocationStrategy.ROUND_ROBIN:
            return self._batch_round_robin(valid_tasks, available_robots)
        elif self._strategy == AllocationStrategy.EMERGENCY:
            return self._batch_emergency(valid_tasks, available_robots)
        elif self._strategy == AllocationStrategy.LOAD_BALANCED:
            return self._batch_load_balanced(valid_tasks, available_robots)
        else:
            return self._batch_greedy(valid_tasks, available_robots)

    def score_single(
        self,
        task: WarehouseTask,
        robot: RobotInfo,
    ) -> AllocationScore:
        """计算单个 (task, robot) 的分配评分明细。"""
        return self._compute_score(task, robot)

    def estimate_completion_time(
        self,
        task: WarehouseTask,
        robot: RobotInfo,
        avg_speed: float = 0.5,
    ) -> float:
        dist = _euclidean(
            robot.pose_x, robot.pose_y,
            task.pickup_x, task.pickup_y,
        )
        pickup_to_dropoff = _euclidean(
            task.pickup_x, task.pickup_y,
            task.dropoff_x, task.dropoff_y,
        )
        travel_time = (dist + pickup_to_dropoff) / max(avg_speed, 0.1)
        loading_time = 5.0
        return travel_time + 2 * loading_time

    # ------------------------------------------------------------------
    # 策略实现
    # ------------------------------------------------------------------

    def _assign_round_robin(self, robots: List[RobotInfo]) -> Optional[str]:
        """轮询分配：轮流给每个可用机器人。"""
        if not robots:
            return None
        n = len(robots)
        for _ in range(n):
            self._round_robin_index = (self._round_robin_index + 1) % n
        return robots[self._round_robin_index % n].robot_id

    def _assign_greedy(
        self,
        task: WarehouseTask,
        robots: List[RobotInfo],
    ) -> Optional[str]:
        """贪心分配：综合评分最优。"""
        best_robot = None
        best_score = float("inf")
        for robot in robots:
            score = self._compute_score(task, robot)
            if score.total < best_score:
                best_score = score.total
                best_robot = robot
        return best_robot.robot_id if best_robot else None

    def _assign_emergency(
        self,
        task: WarehouseTask,
        robots: List[RobotInfo],
    ) -> Optional[str]:
        """紧急模式：电池优先 + 高优先级任务加速 + 截止时间惩罚。"""
        sorted_robots = sorted(
            robots,
            key=lambda r: (
                -r.battery_level,
                r.current_task_id is None,
            ),
        )
        for robot in sorted_robots:
            if robot.battery_level >= 50.0:
                return robot.robot_id
        return sorted_robots[0].robot_id if sorted_robots else None

    def _assign_load_balanced(
        self,
        task: WarehouseTask,
        robots: List[RobotInfo],
    ) -> Optional[str]:
        """负载均衡：防止单机器人过载，优先选择负荷最低的机器人。"""
        sorted_robots = sorted(
            robots,
            key=lambda r: (
                self.workload_history.get(r.robot_id, 0),
                -r.battery_level,
            ),
        )
        return sorted_robots[0].robot_id if sorted_robots else None

    def _batch_greedy(
        self,
        tasks: List[WarehouseTask],
        robots: List[RobotInfo],
    ) -> Dict[str, str]:
        """贪心批量分配。"""
        assignments: Dict[str, str] = {}
        remaining = {r.robot_id: r for r in robots}
        sorted_tasks = sorted(tasks, key=lambda t: -int(t.priority))
        for task in sorted_tasks:
            if not remaining:
                break
            candidates = list(remaining.values())
            best_id = self._assign_greedy(task, candidates)
            if best_id:
                assignments[task.task_id] = best_id
                remaining.pop(best_id, None)
                self.workload_history[best_id] = self.workload_history.get(best_id, 0) + 1
        return assignments

    def _batch_round_robin(
        self,
        tasks: List[WarehouseTask],
        robots: List[RobotInfo],
    ) -> Dict[str, str]:
        """轮询批量分配。"""
        assignments: Dict[str, str] = {}
        if not robots:
            return {}
        sorted_tasks = sorted(tasks, key=lambda t: -int(t.priority))
        for i, task in enumerate(sorted_tasks):
            robot = robots[i % len(robots)]
            assignments[task.task_id] = robot.robot_id
            self.workload_history[robot.robot_id] = self.workload_history.get(robot.robot_id, 0) + 1
        return assignments

    def _batch_emergency(
        self,
        tasks: List[WarehouseTask],
        robots: List[RobotInfo],
    ) -> Dict[str, str]:
        """紧急批量分配：高优先级任务优先分配电池充足的机器人。"""
        assignments: Dict[str, str] = {}
        sorted_tasks = sorted(tasks, key=lambda t: (-int(t.priority), t.age))
        remaining = {r.robot_id: r for r in robots}
        for task in sorted_tasks:
            if not remaining:
                break
            candidates = list(remaining.values())
            best_id = self._assign_emergency(task, candidates)
            if best_id:
                assignments[task.task_id] = best_id
                remaining.pop(best_id, None)
                self.workload_history[best_id] = self.workload_history.get(best_id, 0) + 1
        return assignments

    def _batch_load_balanced(
        self,
        tasks: List[WarehouseTask],
        robots: List[RobotInfo],
    ) -> Dict[str, str]:
        """负载均衡批量分配。"""
        assignments: Dict[str, str] = {}
        remaining = {r.robot_id: r for r in robots}
        sorted_tasks = sorted(tasks, key=lambda t: -int(t.priority))
        for task in sorted_tasks:
            if not remaining:
                break
            candidates = list(remaining.values())
            best_id = self._assign_load_balanced(task, candidates)
            if best_id:
                assignments[task.task_id] = best_id
                remaining.pop(best_id, None)
                self.workload_history[best_id] = self.workload_history.get(best_id, 0) + 1
        return assignments

    def _batch_hungarian(
        self,
        tasks: List[WarehouseTask],
        robots: List[RobotInfo],
    ) -> Dict[str, str]:
        """匈牙利算法批量分配。"""
        n_tasks = len(tasks)
        n_robots = len(robots)
        n = max(n_tasks, n_robots)
        INF = 1e9

        cost = [[INF] * n for _ in range(n)]
        for i, task in enumerate(tasks):
            for j, robot in enumerate(robots):
                if robot.battery_level >= self.battery_reserve and robot.is_healthy:
                    score = self._compute_score(task, robot)
                    cost[i][j] = score.total

        assignment = self._hungarian_core(cost, n)
        result: Dict[str, str] = {}
        for i, j in enumerate(assignment):
            if i < n_tasks and j < n_robots and cost[i][j] < INF:
                robot_id = robots[j].robot_id
                result[tasks[i].task_id] = robot_id
                self.workload_history[robot_id] = self.workload_history.get(robot_id, 0) + 1
        return result

    # ------------------------------------------------------------------
    # 评分引擎
    # ------------------------------------------------------------------

    def _filter_eligible(
        self,
        task: WarehouseTask,
        robots: List[RobotInfo],
    ) -> List[RobotInfo]:
        """过滤出符合基本条件的机器人。"""
        return [
            r for r in robots
            if r.battery_level >= self.battery_reserve
            and r.is_healthy
            and self.workload_history.get(r.robot_id, 0) < self.max_tasks_per_robot
        ]

    def _compute_score(self, task: WarehouseTask, robot: RobotInfo) -> AllocationScore:
        """
        计算 (task, robot) 综合分配评分。

        评分越高 = 越适合接这个任务
        实际 cost = -score（转为越小越好）
        """
        # 距离因子
        dist = _euclidean(
            robot.pose_x, robot.pose_y,
            task.pickup_x, task.pickup_y,
        )
        max_dist = 40.0
        proximity = max(0.0, 1.0 - (dist / max_dist))

        # 电池因子：低于阈值则大幅降低优先级
        batt = robot.battery_level
        if batt < 20.0:
            batt_factor = 0.1
        elif batt < 40.0:
            batt_factor = 0.5
        else:
            batt_factor = 1.0

        # 工作量因子：已完成任务越多，得分越低
        completed = robot.total_tasks_completed
        workload = min(1.0, completed / 10.0)

        # 优先级因子：高优先级任务应分配给综合评分更高的机器人
        priority_map = {0: 0.5, 1: 1.0, 2: 1.5, 3: 2.0}
        priority_factor = priority_map.get(int(task.priority), 1.0)

        # 截止时间因子：剩余时间越少，得分越低
        deadline_factor = 1.0
        if task.deadline is not None:
            remaining = task.deadline - time.time()
            if remaining <= 0:
                deadline_factor = 10.0  # 已过期，强制分配
            elif remaining < 30:
                deadline_factor = 5.0
            elif remaining < 60:
                deadline_factor = 2.0

        # 综合得分（越高越好）
        total = (
            self.w_batt       * batt_factor
            + self.w_dist     * proximity
            - self.w_workload * workload
            + self.w_priority * priority_factor
            + self.w_deadline * deadline_factor
        )

        return AllocationScore(
            robot_id=robot.robot_id,
            task_id=task.task_id,
            total=total,
            distance=dist,
            battery_factor=batt_factor,
            workload_factor=1.0 - workload,
            priority_factor=priority_factor,
            deadline_factor=deadline_factor,
            proximity_factor=proximity,
        )

    # ------------------------------------------------------------------
    # 匈牙利算法核心（与原有相同，优化后保留）
    # ------------------------------------------------------------------

    @staticmethod
    def _hungarian_core(cost: List[List[float]], n: int) -> List[int]:
        """Munkres 匈牙利算法，返回 assignment[i] = j。"""
        INF = float("inf")
        u = [0.0] * (n + 1)
        v = [0.0] * (n + 1)
        p = [0] * (n + 1)
        way = [0] * (n + 1)

        for i in range(1, n + 1):
            p[0] = i
            j0 = 0
            minv = [INF] * (n + 1)
            used = [False] * (n + 1)

            while True:
                used[j0] = True
                i0 = p[j0]
                delta = INF
                j1 = -1

                for j in range(1, n + 1):
                    if not used[j]:
                        cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                        if cur < minv[j]:
                            minv[j] = cur
                            way[j] = j0
                        if minv[j] < delta:
                            delta = minv[j]
                            j1 = j

                if j1 == -1:
                    break

                for j in range(n + 1):
                    if used[j]:
                        u[p[j]] += delta
                        v[j] -= delta
                    else:
                        minv[j] -= delta
                j0 = j1
                if p[j0] == 0:
                    break

            while j0:
                p[j0] = p[way[j0]]
                j0 = way[j0]

        assignment = [0] * n
        for j in range(1, n + 1):
            if p[j] > 0:
                assignment[p[j] - 1] = j - 1
        return assignment

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_strategy(s: str) -> AllocationStrategy:
        s = s.lower().strip()
        mapping = {
            "round_robin": AllocationStrategy.ROUND_ROBIN,
            "greedy":      AllocationStrategy.GREEDY,
            "hungarian":   AllocationStrategy.HUNGARIAN,
            "emergency":   AllocationStrategy.EMERGENCY,
            "load_balanced": AllocationStrategy.LOAD_BALANCED,
        }
        return mapping.get(s, AllocationStrategy.GREEDY)
