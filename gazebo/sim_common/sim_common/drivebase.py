"""
Sim equivalent of the real rover's drivebase node
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Float64MultiArray

# matches urc-2023/src/drivebase/drivebase/drivebase.py so a given joystick
# deflection produces the same wheel speed on the sim as on the real rover
SPEED = 6.28 * 1.5  # rad/s

# order must match the joints list in chassis_bringup/config/chassis.controller.yaml
WHEEL_ORDER = ["fl_wheel", "ml_wheel", "bl_wheel", "fr_wheel", "mr_wheel", "br_wheel"]
LEFT_WHEELS = ("fl_wheel", "ml_wheel", "bl_wheel")
RIGHT_WHEELS = ("fr_wheel", "mr_wheel", "br_wheel")

# cmd_vel geometry from chassis.urdf. The joint names come from the CAD model, which
# faces the other way: base_link +x (the rover's front) points at the "b" wheels, so
# the "l" wheels are on the rover's right
WHEEL_RADIUS = 0.105  # m
HALF_TRACK = 0.267  # m, rotation centre to each side's wheels
# stop if cmd_vel goes quiet, e.g. when Nav2 is killed mid-drive
CMD_VEL_TIMEOUT = Duration(seconds=0.5)


class Drivebase(Node):
    """Main class"""

    def __init__(self):
        super().__init__("drivebase")

        self.wheel_velocity = dict.fromkeys(WHEEL_ORDER, 0.0)
        self.cmd_vel_deadline: Time | None = None

        self.command_pub = self.create_publisher(
            Float64MultiArray, "/forward_velocity_controller/commands", 10
        )

        self.create_subscription(Float32, "move_left_drivebase_side_message", self._on_left, 10)
        self.create_subscription(Float32, "move_right_drivebase_side_message", self._on_right, 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(0.1, self._check_cmd_vel_timeout)

    def _on_left(self, msg: Float32) -> None:
        self.cmd_vel_deadline = None
        self._set_left(msg.data * SPEED)
        self._publish()

    def _on_right(self, msg: Float32) -> None:
        self.cmd_vel_deadline = None
        self._set_right(msg.data * SPEED)
        self._publish()

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Differential drive about base_footprint, the rover's rotation centre"""
        turn = msg.angular.z * HALF_TRACK
        # the "l" wheels are on the rover's right, so they speed up to turn left (+z)
        left = (msg.linear.x + turn) / WHEEL_RADIUS
        right = (msg.linear.x - turn) / WHEEL_RADIUS
        # past the motors' top speed, slow both sides together to keep the curvature
        scale = SPEED / max(abs(left), abs(right), SPEED)
        self._set_left(left * scale)
        self._set_right(right * scale)
        self._publish()
        self.cmd_vel_deadline = self.get_clock().now() + CMD_VEL_TIMEOUT

    def _check_cmd_vel_timeout(self) -> None:
        if self.cmd_vel_deadline is not None and self.get_clock().now() > self.cmd_vel_deadline:
            self.cmd_vel_deadline = None
            self._set_left(0.0)
            self._set_right(0.0)
            self._publish()

    def _set_left(self, vel: float) -> None:
        for wheel in LEFT_WHEELS:
            self.wheel_velocity[wheel] = vel

    def _set_right(self, vel: float) -> None:
        # the right wheels' joint axes point the other way
        for wheel in RIGHT_WHEELS:
            self.wheel_velocity[wheel] = -vel

    def _publish(self) -> None:
        msg = Float64MultiArray()
        msg.data = [self.wheel_velocity[wheel] for wheel in WHEEL_ORDER]
        self.command_pub.publish(msg)


def main(args=None):
    """Main method"""
    rclpy.init(args=args)
    node = Drivebase()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
