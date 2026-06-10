"""
Fleet Load Analyzer — 负荷分析器

核心职责：
1. 实时采集车队状态，量化任务负荷、车辆负载、任务密集度
2. 识别负荷等级（LOW / MODERATE / HIGH / PEAK）
3. 为调度策略引擎提供自适应切换依据

负荷维度：
- 系统级：pending_tasks / available_robots（任务密集度）
- 机器人级：当前任务数、预计完成时间、电池余量、累计行驶距离
"""

from __future__ import annotations

import time
import statistics
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional

from .robot_state_manager import RobotInfo, RobotStateManager
from .task_queue import WarehouseTask, TaskState


class LoadLevel(IntEnum):
    """系统负荷等级，数值越大表示越繁忙。"""
    LOW      = 0
    MODERATE = 1
    HIGH     = 2
    PEAK     = 3


@dataclass
class LoadMetrics:
    """某一时刻的负荷快照。"""
    level:              LoadLevel = LoadLevel.LOW
    pending_count:       int = 0
    active_count:        int = 0
    available_robots:    int = 0
    total_robots:        int = 0
    avg_battery:         float = 100.0
    min_battery:         float = 100.0
    peak_tasks_last_60s: int = 0
    avg_queue_wait:      float = 0.0
    task_density:        float = 0.0
    robot_utilization:   float = 0.0
    timestamp:           float = field(default_factory=time.time)


@dataclass
class RobotLoadProfile:
    """单个机器人的负荷画像。"""
    robot_id:            str
    workload_score:      float = 0.0
    estimated_free_at:   float = 0.0
    task_queue_length:   int = 0
    battery_factor:      float = 1.0
    proximity_factor:    float = 1.0
    priority_factor:     float = 1.0
    total_score:         float = 0.0


class FleetLoadAnalyzer:
    """
    多维度负荷感知与分析器。

    工作流程：
    1. tick() 被定期调用（~0.5 Hz），更新负荷快照
    2. get_load_level() 返回当前系统负荷等级
    3. get_robot_load_profiles() 为任务分配器提供每个机器人的负荷评分
    4. get_recommended_strategy() 返回当前应使用的调度策略
    """

    # 任务密集度阈值（pending_tasks / available_robots）
    DENSITY_LOW:     float = 0.5
    DENSITY_MODERATE: float = 1.2
    DENSITY_HIGH:    float = 2.0
    # 超过 PEAK 阈值时触发紧急策略

    # 机器人利用率阈值
    UTIL_HIGH:       float = 0.75
    UTIL_PEAK:       float = 0.90

    def __init__(self) -> None:
        self._state_mgr: Optional[RobotStateManager] = None
        self._current_metrics = LoadMetrics()
        self._history: List[LoadMetrics] = []
        self._max_history = 120

        self._task_arrival_times: List[float] = []
        self._task_completion_times: List[float] = []
        self._lock__ = __import__("threading").Lock()

    def bind_state_manager(self, state_mgr: RobotStateManager) -> None:
        self._state_mgr = state_mgr

    # ------------------------------------------------------------------
    # 主循环接口
    # ------------------------------------------------------------------

    def tick(
        self,
        pending_tasks: List[WarehouseTask],
        active_tasks: List[WarehouseTask],
        all_robots: List[RobotInfo],
    ) -> LoadMetrics:
        """
        每调度周期调用一次，更新负荷指标并返回当前快照。
        """
        total_robots = len(all_robots)
        available = sum(1 for r in all_robots if r.is_available)
        batteries = [r.battery_level for r in all_robots] if all_robots else [100.0]

        now = time.time()

        pending_count = len(pending_tasks)
        active_count = len(active_tasks)

        # 任务密集度：pending / available（available=0 时取 total）
        divisor = max(available, 1)
        task_density = pending_count / divisor

        # 队列平均等待时间
        ages = [t.age for t in pending_tasks]
        avg_wait = statistics.mean(ages) if ages else 0.0

        # 峰值检测：最近 60 秒到达的任务数
        self._task_arrival_times.append(now)
        self._task_arrival_times = [
            t for t in self._task_arrival_times if now - t <= 60.0
        ]
        peak_count = len(self._task_arrival_times)

        # 机器人利用率
        utilization = self._state_mgr.fleet_utilization() if self._state_mgr else 0.0

        # 计算负荷等级
        level = self._compute_load_level(
            task_density=task_density,
            utilization=utilization,
            pending=pending_count,
            available=available,
            total=total_robots,
        )

        metrics = LoadMetrics(
            level=level,
            pending_count=pending_count,
            active_count=active_count,
            available_robots=available,
            total_robots=total_robots,
            avg_battery=round(statistics.mean(batteries), 1),
            min_battery=round(min(batteries), 1),
            peak_tasks_last_60s=peak_count,
            avg_queue_wait=round(avg_wait, 1),
            task_density=round(task_density, 2),
            robot_utilization=round(utilization, 3),
            timestamp=now,
        )

        self._current_metrics = metrics
        self._history.append(metrics)
        if len(self._history) > self._max_history:
            self._history.pop(0)

        return metrics

    def _compute_load_level(
        self,
        task_density: float,
        utilization: float,
        pending: int,
        available: int,
        total: int,
    ) -> LoadLevel:
        """
        基于多维度加权评分确定负荷等级。

        评分构成：
        - 任务密集度 (40%)
        - 机器人利用率 (30%)
        - 任务积压绝对数量 (30%)
        """
        density_score = 0.0
        if task_density >= self.DENSITY_HIGH:
            density_score = 2.0
        elif task_density >= self.DENSITY_MODERATE:
            density_score = 1.0
        elif task_density >= self.DENSITY_LOW:
            density_score = 0.5
        else:
            density_score = 0.0

        util_score = 0.0
        if utilization >= self.UTIL_PEAK:
            util_score = 2.0
        elif utilization >= self.UTIL_HIGH:
            util_score = 1.0
        else:
            util_score = utilization / self.UTIL_HIGH

        backlog_score = 0.0
        if available == 0 and total > 0:
            backlog_score = 2.0
        elif pending > total * 2:
            backlog_score = 1.5
        elif pending > total:
            backlog_score = 1.0
        else:
            backlog_score = pending / max(total, 1)

        composite = 0.4 * density_score + 0.3 * util_score + 0.3 * backlog_score

        if composite >= 2.0:
            return LoadLevel.PEAK
        elif composite >= 1.5:
            return LoadLevel.HIGH
        elif composite >= 0.7:
            return LoadLevel.MODERATE
        return LoadLevel.LOW

    # ------------------------------------------------------------------
    # 策略推荐
    # ------------------------------------------------------------------

    def get_recommended_strategy(self) -> str:
        """
        根据当前负荷等级推荐最优调度策略。

        策略映射：
        - LOW:     分区轮询（MIN_MIN）
        - MODERATE: 最近距离优先
        - HIGH:    匈牙利最优分配
        - PEAK:    紧急加速模式（带超时惩罚）
        """
        level = self._current_metrics.level
        if level == LoadLevel.PEAK:
            return "emergency"
        elif level == LoadLevel.HIGH:
            return "hungarian"
        elif level == LoadLevel.MODERATE:
            return "greedy"
        return "round_robin"

    # ------------------------------------------------------------------
    # 机器人负荷画像
    # ------------------------------------------------------------------

    def get_robot_load_profiles(
        self,
        robots: List[RobotInfo],
        tasks: List[WarehouseTask],
        avg_speed: float = 0.5,
    ) -> Dict[str, RobotLoadProfile]:
        """
        为每个机器人计算综合负荷评分（供分配器使用）。

        评分因子：
        1. 工作量分数：已完成任务数越多，得分越高（越"累"）
        2. 预计空闲时间：距离当前任务完成还需要的秒数
        3. 电池因子：低于 40% 时降低优先级
        4. 邻近因子：距离任务 pickup 越近，得分越高
        5. 优先级因子：当前任务队列中高优先级任务越多，该机器人越繁忙
        """
        profiles: Dict[str, RobotLoadProfile] = {}
        pending = [t for t in tasks if t.state == TaskState.PENDING]

        # 计算任务优先级加权
        priority_weights = {0: 0.5, 1: 1.0, 2: 1.5, 3: 2.0}

        for robot in robots:
            from .task_queue import TaskPriority
            # 工作量评分（已完成任务数归一化，+ 累计距离）
            completed = robot.total_tasks_completed
            distance = robot.total_distance_traveled
            workload = (completed * 0.1) + (distance * 0.001)

            # 预计空闲时间（估算）
            if robot.current_task_id:
                est_time = 60.0
            else:
                est_time = 0.0

            # 电池因子
            if robot.battery_level < 20.0:
                batt_factor = 0.1
            elif robot.battery_level < 40.0:
                batt_factor = 0.5
            else:
                batt_factor = 1.0

            profile = RobotLoadProfile(
                robot_id=robot.robot_id,
                workload_score=workload,
                estimated_free_at=time.time() + est_time,
                task_queue_length=completed,
                battery_factor=batt_factor,
            )
            profiles[robot.robot_id] = profile

        # 计算邻近因子（距离最近 pending 任务的加权平均）
        for robot in robots:
            profile = profiles[robot.robot_id]
            if not pending:
                profile.proximity_factor = 1.0
            else:
                weighted_dist = 0.0
                total_weight = 0.0
                for task in pending:
                    import math
                    dist = math.hypot(
                        robot.pose_x - task.pickup_x,
                        robot.pose_y - task.pickup_y,
                    )
                    w = priority_weights.get(int(task.priority), 1.0)
                    weighted_dist += dist * w
                    total_weight += w
                avg_dist = weighted_dist / max(total_weight, 1.0)
                max_dist = 30.0
                profile.proximity_factor = max(0.1, 1.0 - (avg_dist / max_dist))

            # 综合得分（越高表示越适合接新任务）
            profile.total_score = (
                profile.battery_factor * 2.0
                + profile.proximity_factor * 1.5
                - profile.workload_score * 0.5
            )

        return profiles

    # ------------------------------------------------------------------
    # 实时查询
    # ------------------------------------------------------------------

    @property
    def current_metrics(self) -> LoadMetrics:
        return self._current_metrics

    def get_load_trend(self, window: int = 10) -> str:
        """返回最近 N 个快照的趋势：'rising', 'falling', 'stable'。"""
        if len(self._history) < 2:
            return "stable"
        recent = self._history[-window:]
        densities = [m.task_density for m in recent]
        if len(densities) < 2:
            return "stable"
        slope = densities[-1] - densities[0]
        if slope > 0.2:
            return "rising"
        elif slope < -0.2:
            return "falling"
        return "stable"

    def get_peak_load_count(self) -> int:
        """返回历史上出现 PEAK 负荷的次数。"""
        return sum(1 for m in self._history if m.level == LoadLevel.PEAK)

    def reset(self) -> None:
        self._history.clear()
        self._task_arrival_times.clear()
        self._task_completion_times.clear()
        self._current_metrics = LoadMetrics()
