"""
Priority-based task queue for the warehouse fleet manager.

Tasks are ordered by priority (CRITICAL > HIGH > NORMAL > LOW) and
within the same priority, by arrival time (FIFO).
"""

from __future__ import annotations

import heapq
import uuid
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional
import time


class TaskPriority(IntEnum):
    LOW      = 0
    NORMAL   = 1
    HIGH     = 2
    CRITICAL = 3


class TaskState(IntEnum):
    PENDING    = 0
    ASSIGNED   = 1
    EXECUTING  = 2
    COMPLETED  = 3
    FAILED     = 4
    CANCELLED  = 5


@dataclass
class WarehouseTask:
    """Represents a single warehouse task."""
    task_id:         str
    task_type:       int             # matches TaskRequest.TASK_* constants
    pickup_x:        float
    pickup_y:        float
    dropoff_x:       float
    dropoff_y:       float
    pickup_zone:     str = ""
    dropoff_zone:    str = ""
    priority:        TaskPriority = TaskPriority.NORMAL
    estimated_weight: float = 0.0
    requester_id:    str = "unknown"
    created_at:      float = field(default_factory=time.time)
    deadline:        Optional[float] = None   # Unix timestamp, None = no deadline

    # Runtime state
    state:           TaskState = TaskState.PENDING
    assigned_robot:  Optional[str] = None
    assigned_at:     Optional[float] = None
    started_at:      Optional[float] = None
    completed_at:    Optional[float] = None
    retry_count:     int = 0
    max_retries:     int = 2

    def __post_init__(self) -> None:
        if not self.task_id:
            self.task_id = str(uuid.uuid4())

    @property
    def age(self) -> float:
        return time.time() - self.created_at

    @property
    def is_overdue(self) -> bool:
        if self.deadline is None:
            return False
        return time.time() > self.deadline

    @property
    def can_retry(self) -> bool:
        return self.retry_count < self.max_retries

    def reset_for_retry(self) -> None:
        self.state = TaskState.PENDING
        self.assigned_robot = None
        self.assigned_at = None
        self.started_at = None
        self.retry_count += 1


@dataclass(order=True)
class _QueueEntry:
    """Heap entry for the priority queue (lower value = higher priority)."""
    sort_key: tuple     # (-priority, created_at)
    task: WarehouseTask = field(compare=False)


class TaskQueue:
    """
    Thread-safe priority queue for warehouse tasks.

    Supports:
    - Priority ordering (CRITICAL first)
    - FIFO within same priority
    - O(log n) insert and pop
    - O(1) lookup by task_id
    - Task state transitions
    """

    def __init__(self) -> None:
        self._heap: List[_QueueEntry] = []
        self._task_map: Dict[str, WarehouseTask] = {}   # task_id -> task
        self._completed: List[WarehouseTask] = []
        self._failed: List[WarehouseTask] = []

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def push(self, task: WarehouseTask) -> None:
        """Add a task to the queue."""
        entry = _QueueEntry(
            sort_key=(-int(task.priority), task.created_at),
            task=task,
        )
        heapq.heappush(self._heap, entry)
        self._task_map[task.task_id] = task

    def pop(self) -> Optional[WarehouseTask]:
        """Remove and return the highest-priority pending task."""
        while self._heap:
            entry = heapq.heappop(self._heap)
            task = entry.task
            if task.state == TaskState.PENDING:
                return task
            # Task was cancelled or already assigned — skip
        return None

    def peek(self) -> Optional[WarehouseTask]:
        """Return highest-priority pending task without removing it."""
        for entry in sorted(self._heap):
            if entry.task.state == TaskState.PENDING:
                return entry.task
        return None

    def get_task(self, task_id: str) -> Optional[WarehouseTask]:
        return self._task_map.get(task_id)

    def cancel_task(self, task_id: str) -> bool:
        """Mark task as cancelled. Returns True if found."""
        task = self._task_map.get(task_id)
        if task is None:
            return False
        if task.state in (TaskState.PENDING, TaskState.ASSIGNED):
            task.state = TaskState.CANCELLED
            return True
        return False

    def mark_assigned(self, task_id: str, robot_id: str) -> bool:
        task = self._task_map.get(task_id)
        if task is None:
            return False
        task.state = TaskState.ASSIGNED
        task.assigned_robot = robot_id
        task.assigned_at = time.time()
        return True

    def mark_executing(self, task_id: str) -> bool:
        task = self._task_map.get(task_id)
        if task is None:
            return False
        task.state = TaskState.EXECUTING
        task.started_at = time.time()
        return True

    def mark_completed(self, task_id: str) -> bool:
        task = self._task_map.get(task_id)
        if task is None:
            return False
        task.state = TaskState.COMPLETED
        task.completed_at = time.time()
        self._completed.append(task)
        return True

    def mark_failed(self, task_id: str, retry: bool = True) -> bool:
        task = self._task_map.get(task_id)
        if task is None:
            return False
        if retry and task.can_retry:
            task.reset_for_retry()
            self.push(task)  # Re-queue with same priority
            return True
        task.state = TaskState.FAILED
        self._failed.append(task)
        return True

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def get_pending_tasks(self) -> List[WarehouseTask]:
        return [e.task for e in self._heap if e.task.state == TaskState.PENDING]

    def get_assigned_tasks(self) -> List[WarehouseTask]:
        return [t for t in self._task_map.values()
                if t.state in (TaskState.ASSIGNED, TaskState.EXECUTING)]

    def get_active_tasks(self) -> List[WarehouseTask]:
        return [t for t in self._task_map.values()
                if t.state in (TaskState.PENDING, TaskState.ASSIGNED, TaskState.EXECUTING)]

    @property
    def pending_count(self) -> int:
        return sum(1 for e in self._heap if e.task.state == TaskState.PENDING)

    @property
    def active_count(self) -> int:
        return len(self.get_active_tasks())

    @property
    def completed_count(self) -> int:
        return len(self._completed)

    @property
    def failed_count(self) -> int:
        return len(self._failed)

    def get_completed_tasks(self) -> List[WarehouseTask]:
        return list(self._completed)

    def get_stats(self) -> Dict:
        completed = self._completed
        avg_time = 0.0
        if completed:
            times = [
                t.completed_at - t.started_at
                for t in completed
                if t.started_at and t.completed_at
            ]
            avg_time = sum(times) / len(times) if times else 0.0

        return {
            "pending":   self.pending_count,
            "active":    self.active_count,
            "completed": self.completed_count,
            "failed":    self.failed_count,
            "avg_completion_time": avg_time,
        }
