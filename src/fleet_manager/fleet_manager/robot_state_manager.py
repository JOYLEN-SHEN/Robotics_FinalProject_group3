"""
Robot state manager for the warehouse fleet.

Tracks real-time state of each robot including pose, battery,
current task, velocity, and heartbeat health monitoring.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple


class RobotState(IntEnum):
    IDLE       = 0
    NAVIGATING = 1
    LOADING    = 2
    UNLOADING  = 3
    CHARGING   = 4
    ERROR      = 5
    WAITING    = 6   # Paused due to conflict resolution


@dataclass
class RobotInfo:
    """Complete state of a single warehouse robot."""
    robot_id:        str
    state:           RobotState = RobotState.IDLE
    battery_level:   float = 100.0          # Percentage [0-100]
    current_task_id: Optional[str] = None
    pose_x:          float = 0.0
    pose_y:          float = 0.0
    pose_yaw:        float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    task_progress:   float = 0.0            # [0.0 - 1.0]
    error_message:   str = ""
    last_heartbeat:  float = field(default_factory=time.time)

    # Battery simulation parameters
    idle_drain_rate:        float = 0.0010  # % per second
    moving_drain_rate:      float = 0.0050  # % per second  (~90min runtime)
    charging_rate:          float = 0.0200  # % per second  (~83min to full)
    low_battery_threshold:  float = 20.0    # % - trigger charging
    critical_battery:       float = 5.0     # % - emergency stop

    # Metrics
    total_distance_traveled: float = 0.0   # meters
    total_tasks_completed:   int = 0
    total_tasks_failed:      int = 0

    @property
    def is_available(self) -> bool:
        """Robot can accept new tasks."""
        return (
            self.state == RobotState.IDLE
            and self.battery_level > self.low_battery_threshold
            and not self.error_message
        )

    @property
    def needs_charging(self) -> bool:
        return self.battery_level <= self.low_battery_threshold

    @property
    def is_critical_battery(self) -> bool:
        return self.battery_level <= self.critical_battery

    @property
    def is_healthy(self) -> bool:
        return self.state != RobotState.ERROR

    @property
    def heartbeat_age(self) -> float:
        return time.time() - self.last_heartbeat

    def update_heartbeat(self) -> None:
        self.last_heartbeat = time.time()

    def simulate_battery(self, dt: float) -> None:
        """Simulate battery drain/charge based on current state."""
        if self.state == RobotState.CHARGING:
            self.battery_level = min(100.0, self.battery_level + self.charging_rate * dt)
        elif self.state == RobotState.NAVIGATING:
            self.battery_level = max(0.0, self.battery_level - self.moving_drain_rate * dt)
        elif self.state in (RobotState.IDLE, RobotState.WAITING):
            self.battery_level = max(0.0, self.battery_level - self.idle_drain_rate * dt)
        # No drain during loading/unloading (minimal movement)

    def update_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        linear_vel: float = 0.0,
        angular_vel: float = 0.0,
    ) -> None:
        dx = x - self.pose_x
        dy = y - self.pose_y
        self.total_distance_traveled += (dx * dx + dy * dy) ** 0.5
        self.pose_x = x
        self.pose_y = y
        self.pose_yaw = yaw
        self.linear_velocity  = linear_vel
        self.angular_velocity = angular_vel

    def to_dict(self) -> Dict:
        return {
            "robot_id":         self.robot_id,
            "state":            int(self.state),
            "state_name":       self.state.name,
            "battery_level":    round(self.battery_level, 1),
            "current_task_id":  self.current_task_id or "",
            "pose":             {"x": self.pose_x, "y": self.pose_y, "yaw": self.pose_yaw},
            "linear_velocity":  self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "task_progress":    self.task_progress,
            "is_available":     self.is_available,
            "needs_charging":   self.needs_charging,
            "error_message":    self.error_message,
            "total_distance":   round(self.total_distance_traveled, 2),
            "tasks_completed":  self.total_tasks_completed,
            "heartbeat_age":    round(self.heartbeat_age, 1),
        }


class RobotStateManager:
    """
    Manages the state of all robots in the fleet.

    Responsibilities:
    - Register and deregister robots
    - Update robot state from ROS2 callbacks
    - Battery simulation (when no real battery sensor)
    - Heartbeat monitoring (detect dead robots)
    - Provide robot availability for task allocation
    """

    HEARTBEAT_TIMEOUT = 10.0   # seconds before robot is considered lost

    def __init__(self, robot_ids: List[str]) -> None:
        self._robots: Dict[str, RobotInfo] = {}
        for robot_id in robot_ids:
            self._robots[robot_id] = RobotInfo(robot_id=robot_id)

    # ------------------------------------------------------------------
    # State Updates
    # ------------------------------------------------------------------

    def update_pose(
        self,
        robot_id: str,
        x: float,
        y: float,
        yaw: float,
        linear_vel: float = 0.0,
        angular_vel: float = 0.0,
    ) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.update_pose(x, y, yaw, linear_vel, angular_vel)
            robot.update_heartbeat()

    def update_state(self, robot_id: str, state: RobotState) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.state = state
            robot.update_heartbeat()

    def update_battery(self, robot_id: str, level: float) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.battery_level = max(0.0, min(100.0, level))

    def update_task_progress(self, robot_id: str, progress: float) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.task_progress = max(0.0, min(1.0, progress))

    def assign_task(self, robot_id: str, task_id: str) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.current_task_id = task_id
            robot.state = RobotState.NAVIGATING
            robot.task_progress = 0.0

    def clear_task(self, robot_id: str, success: bool = True) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.current_task_id = None
            robot.state = RobotState.IDLE
            robot.task_progress = 0.0
            if success:
                robot.total_tasks_completed += 1
            else:
                robot.total_tasks_failed += 1

    def set_error(self, robot_id: str, message: str) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.state = RobotState.ERROR
            robot.error_message = message

    def clear_error(self, robot_id: str) -> None:
        robot = self._robots.get(robot_id)
        if robot:
            robot.state = RobotState.IDLE
            robot.error_message = ""

    # ------------------------------------------------------------------
    # Battery simulation tick (call at ~10Hz)
    # ------------------------------------------------------------------

    def tick_battery(self, dt: float) -> List[str]:
        """
        Simulate battery for all robots.
        Returns list of robot_ids that just crossed the low battery threshold.
        """
        alerts = []
        for robot in self._robots.values():
            prev_level = robot.battery_level
            robot.simulate_battery(dt)
            if (
                prev_level > robot.low_battery_threshold
                and robot.battery_level <= robot.low_battery_threshold
            ):
                alerts.append(robot.robot_id)
        return alerts

    # ------------------------------------------------------------------
    # Heartbeat monitoring
    # ------------------------------------------------------------------

    def check_heartbeats(self) -> List[str]:
        """Return list of robot_ids whose heartbeat has timed out."""
        dead = []
        for robot in self._robots.values():
            if (
                robot.heartbeat_age > self.HEARTBEAT_TIMEOUT
                and robot.state != RobotState.ERROR
            ):
                robot.state = RobotState.ERROR
                robot.error_message = f"Heartbeat timeout ({robot.heartbeat_age:.1f}s)"
                dead.append(robot.robot_id)
        return dead

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_robot(self, robot_id: str) -> Optional[RobotInfo]:
        return self._robots.get(robot_id)

    def get_all_robots(self) -> List[RobotInfo]:
        return list(self._robots.values())

    def get_available_robots(self) -> List[RobotInfo]:
        return [r for r in self._robots.values() if r.is_available]

    def get_robots_needing_charge(self) -> List[RobotInfo]:
        return [
            r for r in self._robots.values()
            if r.needs_charging and r.state != RobotState.CHARGING
        ]

    def get_robot_pose(self, robot_id: str) -> Optional[Tuple[float, float, float]]:
        robot = self._robots.get(robot_id)
        if robot:
            return (robot.pose_x, robot.pose_y, robot.pose_yaw)
        return None

    def fleet_utilization(self) -> float:
        """Fraction of robots actively working."""
        if not self._robots:
            return 0.0
        active = sum(
            1 for r in self._robots.values()
            if r.state in (RobotState.NAVIGATING, RobotState.LOADING, RobotState.UNLOADING)
        )
        return active / len(self._robots)

    def all_healthy(self) -> bool:
        return all(r.is_healthy for r in self._robots.values())

    def get_fleet_summary(self) -> Dict:
        return {
            robot_id: robot.to_dict()
            for robot_id, robot in self._robots.items()
        }
