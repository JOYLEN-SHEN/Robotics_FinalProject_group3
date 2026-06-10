"""
Dashboard ROS2 Node — bridges ROS2 topics to WebSocket.

Subscribes to /fleet_status and /task_results, then forwards
data to the Flask-SocketIO server via a thread-safe queue.
Also provides a ROS2 service client wrapper for task submission
from the dashboard.
"""

from __future__ import annotations

import json
import threading
import uuid
from queue import Queue
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose
from warehouse_msgs.msg import FleetStatus, TaskRequest, TaskResult
from warehouse_msgs.srv import AssignTask, CancelTask


class DashboardNode(Node):
    """ROS2 node that feeds data to the Flask dashboard."""

    def __init__(self, event_queue: Queue) -> None:
        super().__init__("warehouse_dashboard")
        self._queue = event_queue

        # ---- Subscriptions ----
        self.create_subscription(
            FleetStatus,
            "/fleet_status",
            self._on_fleet_status,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )
        self.create_subscription(
            TaskResult,
            "/task_results",
            self._on_task_result,
            10,
        )

        # ---- Publisher for task requests ----
        self._task_pub = self.create_publisher(
            TaskRequest, "/task_requests", 10
        )

        # ---- Service clients ----
        self._cancel_client = self.create_client(CancelTask, "/cancel_task")

        self.get_logger().info("Dashboard node started")

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _on_fleet_status(self, msg: FleetStatus) -> None:
        data = {
            "type": "fleet_status",
            "robots": [],
            "stats": {
                "active_tasks":    msg.active_task_count,
                "pending_tasks":   msg.pending_task_count,
                "completed_tasks": msg.completed_task_count,
                "failed_tasks":    msg.failed_task_count,
                "utilization":     round(msg.fleet_utilization * 100, 1),
                "avg_time":        round(msg.average_task_completion_time, 1),
                "conflicts":       msg.conflict_count,
                "all_healthy":     msg.all_robots_healthy,
            },
        }
        for robot in msg.robots:
            data["robots"].append({
                "id":            robot.robot_id,
                "state":         robot.state,
                "battery":       round(robot.battery_level, 1),
                "task_id":       robot.current_task_id,
                "x":             round(robot.current_pose.position.x, 2),
                "y":             round(robot.current_pose.position.y, 2),
                "progress":      round(robot.task_progress * 100, 1),
                "error":         robot.error_message,
                "linear_vel":    round(robot.linear_velocity, 2),
            })
        self._queue.put(data)

    def _on_task_result(self, msg: TaskResult) -> None:
        self._queue.put({
            "type": "task_result",
            "task_id":  msg.task_id,
            "robot_id": msg.robot_id,
            "success":  msg.success,
            "duration": round(msg.completion_time, 1),
            "distance": round(msg.distance_traveled, 1),
            "failure":  msg.failure_reason,
        })

    # ------------------------------------------------------------------
    # Commands from dashboard
    # ------------------------------------------------------------------

    def submit_task(
        self,
        pickup_x: float, pickup_y: float,
        dropoff_x: float, dropoff_y: float,
        pickup_zone: str = "",
        dropoff_zone: str = "",
        priority: int = 1,
    ) -> str:
        """Publish a new task request. Returns task_id."""
        task_id = str(uuid.uuid4())[:8]
        msg = TaskRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id      = task_id
        msg.task_type    = 0   # PICKUP_DELIVERY
        msg.pickup_location.position.x  = pickup_x
        msg.pickup_location.position.y  = pickup_y
        msg.dropoff_location.position.x = dropoff_x
        msg.dropoff_location.position.y = dropoff_y
        msg.pickup_zone  = pickup_zone
        msg.dropoff_zone = dropoff_zone
        msg.priority     = priority
        msg.requester_id = "dashboard"
        self._task_pub.publish(msg)
        return task_id

    def cancel_task(self, task_id: str) -> bool:
        """Cancel a task via service call."""
        if not self._cancel_client.wait_for_service(timeout_sec=2.0):
            return False
        req = CancelTask.Request()
        req.task_id     = task_id
        req.force_cancel = True
        future = self._cancel_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            return future.result().success
        return False


def spin_ros_node(node: DashboardNode) -> None:
    """Run ROS2 spinning in a background thread."""
    rclpy.spin(node)


def main(args=None) -> None:
    """Entry point: start ROS2 node + Flask server."""
    rclpy.init(args=args)

    from .app import create_app

    event_queue: Queue = Queue()
    node = DashboardNode(event_queue)

    # ROS2 in background thread
    ros_thread = threading.Thread(
        target=spin_ros_node, args=(node,), daemon=True
    )
    ros_thread.start()

    # Flask app in main thread
    app, socketio = create_app(node, event_queue)
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
