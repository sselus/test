"""Simple ROS2 patrol node."""

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_node")
        self._waypoints: List[str] = ["dock", "hallway", "lab"]
        self._index = 0
        self._publisher = self.create_publisher(String, "patrol/status", 10)
        self._timer = self.create_timer(2.0, self._tick)

    def _tick(self) -> None:
        msg = String()
        msg.data = f"heading_to={self._waypoints[self._index]}"
        self._publisher.publish(msg)
        self._index = (self._index + 1) % len(self._waypoints)


def main() -> None:
    rclpy.init()
    node = PatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
