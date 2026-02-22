"""Simple ROS2 patrol node."""

from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .patrol_controller import PatrolController
from .safety_monitor import DebouncedStop
from .waypoint_loader import load_waypoints


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_node")
        self.declare_parameter("waypoint_file", "")
        self._waypoints: List[str] = self._resolve_waypoints()
        self._controller = PatrolController(self._waypoints)
        self._safety = DebouncedStop()
        self._publisher = self.create_publisher(String, "patrol/status", 10)
        self._stop_sub = self.create_subscription(Bool, "patrol/stop", self._on_stop, 10)
        self._timer = self.create_timer(2.0, self._tick)
        self.get_logger().info(f"loaded_waypoints={self._waypoints}")

    def _resolve_waypoints(self) -> List[str]:
        waypoint_file = self.get_parameter("waypoint_file").value
        if waypoint_file:
            return load_waypoints(str(waypoint_file))

        default_file = Path(__file__).resolve().parent.parent / "config" / "waypoints.yaml"
        return load_waypoints(str(default_file))

    def _on_stop(self, msg: Bool) -> None:
        state = self._safety.update(bool(msg.data))
        self.get_logger().warning(f"event=safety_update active={state}")

    def _tick(self) -> None:
        if self._safety.active:
            self.get_logger().warning("event=patrol_paused reason=emergency_stop")
            return

        target = self._controller.current_target()
        msg = String()
        msg.data = f"heading_to={target}"
        self._publisher.publish(msg)
        self._controller.advance()
        self.get_logger().info(
            f"event=patrol_transition waypoint={target} total={self._controller.metrics.transitions}"
        )


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
