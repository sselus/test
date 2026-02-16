"""CLI command to publish emergency stop messages."""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class StopPublisher(Node):
    def __init__(self) -> None:
        super().__init__("stop_cmd")
        self._pub = self.create_publisher(Bool, "patrol/stop", 10)

    def publish(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._pub.publish(msg)
        self.get_logger().info(f"published_stop={active}")


def main() -> None:
    rclpy.init()
    node = StopPublisher()
    try:
        value = len(sys.argv) > 1 and sys.argv[1].lower() in {"1", "true", "on", "stop"}
        node.publish(value)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
