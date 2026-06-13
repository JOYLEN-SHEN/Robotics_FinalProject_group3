"""
Fleet Manager ROS2 Node — 动态调度核心

完全重构，替代原有实现。

核心架构变化：
1. 集成 FleetLoadAnalyzer：实时负荷感知 + 策略自动切换
2. 集成 AdaptiveTaskAllocator：多策略任务分配
3. 替换 CBS 为 SimpleConflictResolver：简单规避替代复杂约束搜索
4. 新增动态策略切换：LOW→轮询, MODERATE→贪心, HIGH→匈牙利, PEAK→紧急


标准模拟流程：
1. 启动项目 → 加载AGV小车模型与仓储地图环境
2. 开启SLAM建图 → 手动/自动操纵小车完成仓储环境扫描建图
3. 发布货物搬运任务 → 多台小车执行动态调度完成自动化货运模拟
"""

from __future__ import annotations

import time
import threading
import uuid
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Time as RosTime

from warehouse_msgs.msg import (
    FleetStatus, RobotStatus, TaskRequest, TaskResult
)
from warehouse_msgs.srv import AssignTask, CancelTask, GetRobotStatus

from .robot_state_manager import RobotStateManager, RobotState, RobotInfo
from .task_queue import TaskQueue, WarehouseTask, TaskPriority, TaskState
from .adaptive_task_allocator import AdaptiveTaskAllocator, AllocationStrategy
from .simple_conflict_resolver import (
    SimpleConflictResolver, AvoidanceAction, AvoidanceDecision
)
from .fleet_load_analyzer import FleetLoadAnalyzer, LoadLevel, LoadMetrics
from .warehouse_graph import WarehouseGraph


ROBOT_NAMES = ["robot_1", "robot_2", "robot_3", "robot_4"]

LOADING_DURATION   = 3.0
UNLOADING_DURATION = 3.0

BATTERY_TICK_HZ = 10.0


def _euler_to_quaternion(yaw: float) -> Quaternion:
    import math
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _ros_time_from_float(t: float) -> RosTime:
    msg = RosTime()
    msg.sec     = int(t)
    msg.nanosec = int((t % 1) * 1e9)
    return msg


class FleetManagerNode(Node):
    """多AGV动态调度系统主节点。"""

    def __init__(self) -> None:
        super().__init__("fleet_manager")

        # ---- Parameters ----
        self.declare_parameter("robot_names",         ROBOT_NAMES)
        self.declare_parameter("fleet_status_hz",     1.0)
        self.declare_parameter("allocation_strategy", "greedy")
        self.declare_parameter("auto_adapt_strategy",  True)
        self.declare_parameter("battery_threshold",   20.0)
        self.declare_parameter("task_timeout",        120.0)
        self.declare_parameter("heartbeat_timeout",   10.0)
        self.declare_parameter("conflict_check_hz",    2.0)
        self.declare_parameter("load_analysis_hz",     0.5)

        robot_names      = self.get_parameter("robot_names").value
        status_hz       = self.get_parameter("fleet_status_hz").value
        strategy        = self.get_parameter("allocation_strategy").value
        auto_adapt      = self.get_parameter("auto_adapt_strategy").value

        # ---- 核心组件初始化 ----
        self.graph     = WarehouseGraph()
        self.state_mgr = RobotStateManager(robot_names)
        self.task_q    = TaskQueue()
        self.allocator = AdaptiveTaskAllocator(strategy=strategy, auto_adapt=auto_adapt)
        self.conflict_resolver = SimpleConflictResolver(self.graph)
        self.load_analyzer = FleetLoadAnalyzer()
        self.load_analyzer.bind_state_manager(self.state_mgr)

        # 状态
        self._active_goals: Dict[str, object] = {}
        self._avoidance_decisions: Dict[str, AvoidanceDecision] = {}
        self._lock = threading.Lock()

        # 位置历史（死锁检测）
        self._pos_history: Dict[str, List[Tuple[float, float]]] = {
            r: [] for r in robot_names
        }

        # 机器人当前任务截止时间（用于冲突决策）
        self._robot_deadlines: Dict[str, Optional[float]] = {
            r: None for r in robot_names
        }

        # 策略切换历史（用于日志）
        self._last_strategy_label = ""

        # ---- Callback groups ----
        self._cb_group_reentrant = ReentrantCallbackGroup()
        self._cb_group_exclusive = MutuallyExclusiveCallbackGroup()

        # ---- Publishers ----
        self._fleet_status_pub = self.create_publisher(
            FleetStatus, "/fleet_status",
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )
        self._task_result_pub = self.create_publisher(TaskResult, "/task_results", 10)

        # ---- Subscribers ----
        self.create_subscription(
            TaskRequest, "/task_requests",
            self._on_task_request, 10,
            callback_group=self._cb_group_reentrant,
        )

        for robot_name in robot_names:
            self.create_subscription(
                Odometry,
                f"/{robot_name}/odom",
                lambda msg, rn=robot_name: self._on_odom(rn, msg),
                rclpy.qos.qos_profile_sensor_data,
                callback_group=self._cb_group_reentrant,
            )

        # ---- Services ----
        self.create_service(
            AssignTask, "/assign_task",
            self._srv_assign_task,
            callback_group=self._cb_group_exclusive,
        )
        self.create_service(
            CancelTask, "/cancel_task",
            self._srv_cancel_task,
            callback_group=self._cb_group_exclusive,
        )
        self.create_service(
            GetRobotStatus, "/get_robot_status",
            self._srv_get_robot_status,
            callback_group=self._cb_group_exclusive,
        )

        # ---- Nav2 Action Clients ----
        self._nav_clients: Dict[str, ActionClient] = {}
        for robot_name in robot_names:
            client = ActionClient(
                self, NavigateToPose,
                f"/{robot_name}/navigate_to_pose",
                callback_group=self._cb_group_reentrant,
            )
            self._nav_clients[robot_name] = client

        # ---- Timers ----
        self.create_timer(
            1.0 / status_hz,
            self._publish_fleet_status,
            callback_group=self._cb_group_exclusive,
        )
        self.create_timer(
            0.5,
            self._allocation_tick,
            callback_group=self._cb_group_exclusive,
        )
        self.create_timer(
            1.0 / BATTERY_TICK_HZ,
            self._battery_tick,
            callback_group=self._cb_group_exclusive,
        )
        self.create_timer(
            1.0 / self.get_parameter("conflict_check_hz").value,
            self._conflict_tick,
            callback_group=self._cb_group_exclusive,
        )
        self.create_timer(
            1.0 / self.get_parameter("load_analysis_hz").value,
            self._load_analysis_tick,
            callback_group=self._cb_group_exclusive,
        )
        self.create_timer(
            5.0,
            self._health_check,
            callback_group=self._cb_group_exclusive,
        )

        self.get_logger().info(
            f"Fleet Manager 启动 | 机器人数量: {len(robot_names)} | "
            f"策略: {strategy} | 自动适应: {auto_adapt}"
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _on_odom(self, robot_id: str, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        import math
        yaw = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z),
        )
        self.state_mgr.update_pose(
            robot_id, pos.x, pos.y, yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )
        history = self._pos_history[robot_id]
        history.append((pos.x, pos.y))
        if len(history) > 40:
            history.pop(0)

    def _on_task_request(self, msg: TaskRequest) -> None:
        task = WarehouseTask(
            task_id       = msg.task_id or str(uuid.uuid4()),
            task_type     = msg.task_type,
            pickup_x      = msg.pickup_location.position.x,
            pickup_y      = msg.pickup_location.position.y,
            dropoff_x     = msg.dropoff_location.position.x,
            dropoff_y     = msg.dropoff_location.position.y,
            pickup_zone   = msg.pickup_zone,
            dropoff_zone  = msg.dropoff_zone,
            priority      = TaskPriority(min(int(msg.priority), 3)),
            requester_id  = msg.requester_id,
        )
        self.task_q.push(task)
        self.get_logger().info(
            f"[任务到达] {task.task_id} | "
            f"{task.pickup_zone} → {task.dropoff_zone} | "
            f"优先级: {task.priority.name} | "
            f"当前策略: {self.allocator.get_strategy_label()}"
        )

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _srv_assign_task(
        self, request: AssignTask.Request, response: AssignTask.Response
    ) -> AssignTask.Response:
        task = WarehouseTask(
            task_id     = request.task.task_id or str(uuid.uuid4()),
            task_type   = request.task.task_type,
            pickup_x    = request.task.pickup_location.position.x,
            pickup_y    = request.task.pickup_location.position.y,
            dropoff_x   = request.task.dropoff_location.position.x,
            dropoff_y   = request.task.dropoff_location.position.y,
            pickup_zone = request.task.pickup_zone,
            dropoff_zone= request.task.dropoff_zone,
            priority    = TaskPriority(min(int(request.task.priority), 3)),
        )
        available = self.state_mgr.get_available_robots()
        robot_id  = self.allocator.assign_task(task, available)

        if robot_id is None:
            response.success = False
            response.message = "No available robots"
            return response

        robot = self.state_mgr.get_robot(robot_id)
        self.task_q.push(task)
        self.task_q.mark_assigned(task.task_id, robot_id)
        self.state_mgr.assign_task(robot_id, task.task_id)
        self._robot_deadlines[robot_id] = task.deadline

        response.success = True
        response.assigned_robot_id = robot_id
        response.estimated_time = self.allocator.estimate_completion_time(task, robot)
        response.message = f"Assigned to {robot_id}"

        self.get_logger().info(
            f"[任务分配] {task.task_id} → {robot_id} | "
            f"策略: {self.allocator.get_strategy_label()}"
        )
        threading.Thread(
            target=self._execute_task, args=(robot_id, task), daemon=True
        ).start()
        return response

    def _srv_cancel_task(
        self, request: CancelTask.Request, response: CancelTask.Response
    ) -> CancelTask.Response:
        task = self.task_q.get_task(request.task_id)
        if task is None:
            response.success = False
            response.message = "Task not found"
            return response

        robot_id = task.assigned_robot
        cancelled = self.task_q.cancel_task(request.task_id)

        if cancelled and robot_id:
            self.state_mgr.clear_task(robot_id, success=False)
            self._cancel_nav_goal(robot_id)
            self._robot_deadlines.pop(robot_id, None)

        response.success  = cancelled
        response.robot_id = robot_id or ""
        response.message = "Cancelled" if cancelled else "Could not cancel"
        return response

    def _srv_get_robot_status(
        self, request: GetRobotStatus.Request, response: GetRobotStatus.Response
    ) -> GetRobotStatus.Response:
        if request.robot_id:
            robot = self.state_mgr.get_robot(request.robot_id)
            robots = [robot] if robot else []
        else:
            robots = self.state_mgr.get_all_robots()

        response.success = bool(robots)
        response.robots  = [self._robot_info_to_msg(r) for r in robots]
        response.message = "" if robots else f"Robot '{request.robot_id}' not found"
        return response

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------

    def _allocation_tick(self) -> None:
        """尝试将待分配任务分配给可用机器人（自动调度循环）。"""
        pending = self.task_q.get_pending_tasks()
        if not pending:
            return

        available = self.state_mgr.get_available_robots()
        if not available:
            return

        assignments = self.allocator.batch_assign(pending, available)
        for task_id, robot_id in assignments.items():
            task  = self.task_q.get_task(task_id)
            robot = self.state_mgr.get_robot(robot_id)
            if task is None or robot is None:
                continue

            self.task_q.mark_assigned(task_id, robot_id)
            self.state_mgr.assign_task(robot_id, task_id)
            self._robot_deadlines[robot_id] = task.deadline

            self.get_logger().info(
                f"[自动分配] {task_id} → {robot_id} | "
                f"{task.pickup_zone} → {task.dropoff_zone} | "
                f"策略: {self.allocator.get_strategy_label()}"
            )
            threading.Thread(
                target=self._execute_task, args=(robot_id, task), daemon=True
            ).start()

    def _battery_tick(self) -> None:
        dt = 1.0 / BATTERY_TICK_HZ
        low_battery_robots = self.state_mgr.tick_battery(dt)

        for robot_id in low_battery_robots:
            robot = self.state_mgr.get_robot(robot_id)
            if robot and robot.state == RobotState.IDLE:
                self.get_logger().warn(
                    f"[电量警告] {robot_id}: {robot.battery_level:.1f}% → 前往充电站"
                )
                self._send_to_charger(robot_id)

    def _conflict_tick(self) -> None:
        """冲突检测与规避决策循环。"""
        positions: Dict[str, Tuple[float, float]] = {}
        states: Dict[str, str] = {}

        for robot in self.state_mgr.get_all_robots():
            if robot.state in (RobotState.NAVIGATING, RobotState.IDLE):
                positions[robot.robot_id] = (robot.pose_x, robot.pose_y)
                states[robot.robot_id] = robot.state.name

        if len(positions) < 2:
            return

        decisions = self.conflict_resolver.check_conflicts(
            positions, states, self._robot_deadlines
        )

        self._avoidance_decisions = decisions

        # 应用避让动作
        for robot_id, decision in decisions.items():
            if decision.action == AvoidanceAction.STOP:
                self.get_logger().warn(
                    f"[冲突避让] {robot_id} 停止等待 {decision.target_robot}"
                )
            elif decision.action == AvoidanceAction.SLOWDOWN:
                self.get_logger().info(
                    f"[冲突预警] {robot_id} 减速让行 {decision.target_robot}"
                )

        # 死锁检测
        deadlocked = self.conflict_resolver.detect_deadlock(self._pos_history)
        for robot_id in deadlocked:
            if robot_id not in self._stuck_robots:
                self._stuck_robots.add(robot_id)
                self.get_logger().warn(
                    f"[死锁检测] {robot_id} 陷入停滞，触发重新规划"
                )
                self._cancel_nav_goal(robot_id)

    def _load_analysis_tick(self) -> None:
        """负荷分析 + 策略自适应切换。"""
        pending = self.task_q.get_pending_tasks()
        active  = self.task_q.get_active_tasks()
        robots  = self.state_mgr.get_all_robots()

        metrics = self.load_analyzer.tick(pending, active, robots)
        recommended = self.load_analyzer.get_recommended_strategy()

        if self.allocator._auto_adapt:
            current_label = self.allocator.get_strategy_label()
            if recommended != self.allocator._strategy.value:
                self.allocator.set_strategy(recommended)
                new_label = self.allocator.get_strategy_label()
                if new_label != self._last_strategy_label:
                    self.get_logger().info(
                        f"[策略切换] {self._last_strategy_label} → {new_label} | "
                        f"负荷: {metrics.level.name} | "
                        f"pending={metrics.pending_count} | "
                        f"available={metrics.available_robots} | "
                        f"utilization={metrics.robot_utilization:.1%}"
                    )
                    self._last_strategy_label = new_label

    def _health_check(self) -> None:
        dead = self.state_mgr.check_heartbeats()
        for robot_id in dead:
            self.get_logger().error(f"[健康检查] {robot_id}: 心跳超时 → ERROR")

    def _publish_fleet_status(self) -> None:
        now = self.get_clock().now().to_msg()
        stats    = self.task_q.get_stats()
        metrics  = self.load_analyzer.current_metrics
        conflict = self.conflict_resolver.get_metrics()

        msg = FleetStatus()
        msg.header.stamp    = now
        msg.header.frame_id = "map"
        msg.robots = [
            self._robot_info_to_msg(r)
            for r in self.state_mgr.get_all_robots()
        ]
        msg.active_task_count    = stats["active"]
        msg.pending_task_count   = stats["pending"]
        msg.completed_task_count = stats["completed"]
        msg.failed_task_count   = stats["failed"]
        msg.fleet_utilization    = self.state_mgr.fleet_utilization()
        msg.average_task_completion_time = stats["avg_completion_time"]
        msg.conflict_count       = conflict["total_conflicts"]
        msg.all_robots_healthy   = self.state_mgr.all_healthy()

        self._fleet_status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Task execution
    # ------------------------------------------------------------------

    def _execute_task(self, robot_id: str, task: WarehouseTask) -> None:
        self.task_q.mark_executing(task.task_id)
        start_time = time.time()

        try:
            # Phase 1: Navigate to pickup
            self.get_logger().info(
                f"[执行中] {robot_id} → 取货点 ({task.pickup_x:.1f}, {task.pickup_y:.1f})"
            )
            self.state_mgr.update_state(robot_id, RobotState.NAVIGATING)
            success = self._navigate_to(robot_id, task.pickup_x, task.pickup_y, task.task_id)
            if not success:
                raise RuntimeError("导航至取货点失败")

            # Phase 2: Loading
            self.get_logger().info(f"[装货中] {robot_id}")
            self.state_mgr.update_state(robot_id, RobotState.LOADING)
            time.sleep(LOADING_DURATION)

            # Phase 3: Navigate to dropoff
            self.get_logger().info(
                f"[执行中] {robot_id} → 卸货点 ({task.dropoff_x:.1f}, {task.dropoff_y:.1f})"
            )
            self.state_mgr.update_state(robot_id, RobotState.NAVIGATING)
            success = self._navigate_to(robot_id, task.dropoff_x, task.dropoff_y, task.task_id)
            if not success:
                raise RuntimeError("导航至卸货点失败")

            # Phase 4: Unloading
            self.get_logger().info(f"[卸货中] {robot_id}")
            self.state_mgr.update_state(robot_id, RobotState.UNLOADING)
            time.sleep(UNLOADING_DURATION)

            elapsed = time.time() - start_time
            robot   = self.state_mgr.get_robot(robot_id)
            self.task_q.mark_completed(task.task_id)
            self.state_mgr.clear_task(robot_id, success=True)
            self._robot_deadlines.pop(robot_id, None)

            self._publish_task_result(
                task_id=task.task_id,
                robot_id=robot_id,
                success=True,
                elapsed=elapsed,
                distance=robot.total_distance_traveled if robot else 0.0,
            )
            self.get_logger().info(
                f"[任务完成] {task.task_id} | {robot_id} | "
                f"耗时 {elapsed:.1f}s | 策略: {self.allocator.get_strategy_label()}"
            )

        except Exception as e:
            self.get_logger().error(f"[任务失败] {task.task_id} @ {robot_id}: {e}")
            self.task_q.mark_failed(task.task_id, retry=True)
            self.state_mgr.clear_task(robot_id, success=False)
            self._robot_deadlines.pop(robot_id, None)
            self._publish_task_result(
                task_id=task.task_id,
                robot_id=robot_id,
                success=False,
                elapsed=time.time() - start_time,
                distance=0.0,
                failure_reason=str(e),
            )

    def _navigate_to(
        self,
        robot_id: str,
        x: float,
        y: float,
        task_id: str,
        timeout: float = 90.0,
    ) -> bool:
        nav_client = self._nav_clients.get(robot_id)
        if nav_client is None:
            return False

        if not nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"{robot_id}: Nav2 服务器未就绪")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = _euler_to_quaternion(0.0)

        future = nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.result() is None:
            self.get_logger().warn(f"{robot_id}: 目标被拒绝")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"{robot_id}: 目标未被接受")
            return False

        with self._lock:
            self._active_goals[robot_id] = goal_handle

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        with self._lock:
            self._active_goals.pop(robot_id, None)

        if not result_future.done():
            self.get_logger().warn(f"{robot_id}: 导航超时")
            return False

        from action_msgs.msg import GoalStatus
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def _cancel_nav_goal(self, robot_id: str) -> None:
        with self._lock:
            goal_handle = self._active_goals.pop(robot_id, None)
        if goal_handle:
            goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # Charging
    # ------------------------------------------------------------------

    def _send_to_charger(self, robot_id: str) -> None:
        robot = self.state_mgr.get_robot(robot_id)
        if robot is None:
            return

        charger = self.graph.nearest_free_charging_station(robot.pose_x, robot.pose_y)
        if charger is None:
            self.get_logger().warn(f"{robot_id}: 无可用充电站")
            return

        self.graph.reserve_zone(charger.name, robot_id)
        self.state_mgr.update_state(robot_id, RobotState.CHARGING)

        def _charge() -> None:
            self._navigate_to(
                robot_id, charger.center.x, charger.center.y, "charging"
            )
            while True:
                r = self.state_mgr.get_robot(robot_id)
                if r is None or r.battery_level >= 95.0:
                    break
                time.sleep(1.0)
            self.graph.release_zone(charger.name, robot_id)
            self.state_mgr.update_state(robot_id, RobotState.IDLE)
            self.get_logger().info(f"[充电完成] {robot_id} @ {charger.name}")

        threading.Thread(target=_charge, daemon=True).start()

    # ------------------------------------------------------------------
    # Message builders
    # ------------------------------------------------------------------

    def _robot_info_to_msg(self, robot: RobotInfo) -> RobotStatus:
        msg = RobotStatus()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.robot_id         = robot.robot_id
        msg.state            = int(robot.state)
        msg.battery_level    = robot.battery_level
        msg.current_task_id  = robot.current_task_id or ""
        msg.current_pose.position.x = robot.pose_x
        msg.current_pose.position.y = robot.pose_y
        msg.linear_velocity  = robot.linear_velocity
        msg.angular_velocity= robot.angular_velocity
        msg.task_progress    = robot.task_progress
        msg.error_message    = robot.error_message
        msg.last_heartbeat   = _ros_time_from_float(robot.last_heartbeat)
        return msg

    def _publish_task_result(
        self,
        task_id: str,
        robot_id: str,
        success: bool,
        elapsed: float,
        distance: float,
        failure_reason: str = "",
    ) -> None:
        msg = TaskResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id         = task_id
        msg.robot_id        = robot_id
        msg.success         = success
        msg.completion_time = elapsed
        msg.distance_traveled = distance
        msg.failure_reason  = failure_reason
        self._task_result_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetManagerNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
