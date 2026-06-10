#!/usr/bin/env python3
"""
Relay per-robot namespaced TF topics to the global /tf and /tf_static.

Nav2 with use_namespace=true remaps all TF I/O to /robot_N/tf, so RViz2
(which subscribes to the global /tf) cannot see any transforms.  This node
bridges the gap without modifying Nav2 or the URDF.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)
from tf2_msgs.msg import TFMessage


ROBOT_NAMES = ["robot_1", "robot_2", "robot_3", "robot_4"]

_TF_QOS = QoSProfile(depth=100)

_TF_STATIC_QOS = QoSProfile(
    depth=100,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class TFRelay(Node):
    """Subscribe to /robot_N/tf[_static] and republish on /tf[_static].

    Dynamic transforms are forwarded immediately.
    Static transforms are accumulated so that late-joining subscribers
    (e.g. RViz) receive the full set on connect — not just the last batch.
    """

    def __init__(self) -> None:
        super().__init__("tf_relay")

        self._tf_pub = self.create_publisher(TFMessage, "/tf", _TF_QOS)
        self._tf_static_pub = self.create_publisher(
            TFMessage, "/tf_static", _TF_STATIC_QOS
        )

        # Keyed by (parent_frame, child_frame) so duplicates are overwritten
        self._static_transforms: dict = {}

        for robot in ROBOT_NAMES:
            self.create_subscription(
                TFMessage,
                f"/{robot}/tf",
                self._tf_pub.publish,
                _TF_QOS,
            )
            self.create_subscription(
                TFMessage,
                f"/{robot}/tf_static",
                self._on_static_tf,
                _TF_STATIC_QOS,
            )

        self.get_logger().info(
            f"tf_relay: bridging {ROBOT_NAMES} → /tf and /tf_static"
        )

    def _on_static_tf(self, msg: TFMessage) -> None:
        """Accumulate and republish all static transforms as one message."""
        for transform in msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self._static_transforms[key] = transform
        combined = TFMessage(transforms=list(self._static_transforms.values()))
        self._tf_static_pub.publish(combined)


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = TFRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
