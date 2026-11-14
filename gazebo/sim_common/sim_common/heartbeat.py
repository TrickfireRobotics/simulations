"""Simulation responder for Mission Control's heartbeat protocol."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class HeartbeatResponder(Node):
    """Acknowledge heartbeats using the same topics as the real rover."""

    def __init__(self):
        super().__init__("heartbeat_node")
        self.publisher = self.create_publisher(Bool, "/hbr", 10)
        self.create_subscription(Bool, "/heartbeat", self._on_heartbeat, 10)

    def _on_heartbeat(self, _message: Bool) -> None:
        self.publisher.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatResponder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
