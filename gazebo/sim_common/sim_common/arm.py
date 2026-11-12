"""
Sim equivalent of the real rover's arm node - jogs the arm's joint_trajectory_controller
from the mission-control webbridge, the same way drivebase.py does for the chassis.

Replaces joint_gui.py (the tkinter "tinker UI") as the normal way to drive the arm.
"""

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# must match the joints list in arm_bringup/config/arm.controller.yaml and
# rover_bringup/config/rover.controller.yaml
JOINT_LIMITS = {
    "baseplate_left_right": (-3.14, 3.14),
    "shoulder_up_down": (-1.61808, 0.912649),
    "elbow_up_down": (-3.14, 3.14),
    "gripper_up_down": (-3.14, 3.14),
    "gripper_left_right": (-3.14, 3.14),
    "claw_open_close": (-0.80, 0.80),
}
JOINT_ORDER = list(JOINT_LIMITS)

# left stick Y drives both the shoulder and elbow together (one "arm up/down" motion)
# TEMP: both joints get the same signed delta - this moves the end effector up/down
# only approximately, not a true IK mix. Re-tune signs once this is driven in sim.
AXIS_TO_JOINTS = {
    "arm_left_joy_x": ("baseplate_left_right",),
    "arm_left_joy_y": ("shoulder_up_down", "elbow_up_down"),
    "arm_right_joy_x": ("gripper_left_right",),
    "arm_right_joy_y": ("gripper_up_down",),
}

JOINT_SPEED = 1.0  # rad/s, for the joystick-jogged joints
CLAW_SPEED = 0.3  # rad/s, slower since the claw should close/open "slowly"
UPDATE_PERIOD_S = 0.05  # 20 Hz
TRAJECTORY_DURATION_S = 0.1

TRAJECTORY_TOPIC = "/joint_trajectory_controller/joint_trajectory"


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class Arm(Node):
    """Main class"""

    def __init__(self) -> None:
        super().__init__("arm")

        self.target_positions: dict[str, float] = dict.fromkeys(JOINT_ORDER, 0.0)
        self._seen_joints: set[str] = set()
        self._positions_initialized = False

        self.axis_values: dict[str, float] = dict.fromkeys(AXIS_TO_JOINTS, 0.0)
        self.claw_close = 0.0
        self.claw_open = 0.0

        self.trajectory_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        for topic in AXIS_TO_JOINTS:
            self.create_subscription(Float32, topic, self._make_axis_callback(topic), 10)

        self.create_subscription(Float32, "arm_gripper_close", self._on_claw_close, 10)
        self.create_subscription(Float32, "arm_gripper_open", self._on_claw_open, 10)

        self.create_timer(UPDATE_PERIOD_S, self._on_timer)

    def _make_axis_callback(self, topic: str):
        def callback(msg: Float32) -> None:
            self.axis_values[topic] = msg.data

        return callback

    def _on_claw_close(self, msg: Float32) -> None:
        self.claw_close = msg.data

    def _on_claw_open(self, msg: Float32) -> None:
        self.claw_open = msg.data

    def _on_joint_states(self, msg: JointState) -> None:
        """Seed target_positions from the arm's actual starting pose, so the first
        jog command moves from where the arm really is instead of snapping to 0."""
        if self._positions_initialized:
            return
        for name, position in zip(msg.name, msg.position):
            if name in self.target_positions:
                self.target_positions[name] = position
                self._seen_joints.add(name)
        if self._seen_joints >= set(JOINT_ORDER):
            self._positions_initialized = True

    def _step_joint(self, joint: str, rate: float) -> float:
        """Advance one joint's target by `rate` (rad/s) for one tick, clamped to its
        limits. Returns the feedforward velocity to report for this tick: `rate`, or 0
        if the limit clamped the move (so the spline aims to stop there instead of
        planning to drive through it)."""
        lo, hi = JOINT_LIMITS[joint]
        unclamped = self.target_positions[joint] + rate * UPDATE_PERIOD_S
        self.target_positions[joint] = _clamp(unclamped, lo, hi)
        return rate if self.target_positions[joint] == unclamped else 0.0

    def _on_timer(self) -> None:
        if not self._positions_initialized:
            return

        velocities: dict[str, float] = dict.fromkeys(JOINT_ORDER, 0.0)
        moved = False

        for topic, joints in AXIS_TO_JOINTS.items():
            value = self.axis_values[topic]
            if value == 0.0:
                continue
            moved = True
            rate = value * JOINT_SPEED
            for joint in joints:
                velocities[joint] = self._step_joint(joint, rate)

        claw_rate = (self.claw_open - self.claw_close) * CLAW_SPEED
        if claw_rate != 0.0:
            moved = True
            velocities["claw_open_close"] = self._step_joint("claw_open_close", claw_rate)

        if moved:
            self._publish_target(velocities)

    def _publish_target(self, velocities: dict[str, float]) -> None:
        msg = JointTrajectory()
        msg.joint_names = JOINT_ORDER
        point = JointTrajectoryPoint()
        point.positions = [self.target_positions[joint] for joint in JOINT_ORDER]
        # feed forward the commanded velocity so the controller plans continuous motion
        # instead of braking to a stop at every republished waypoint - that stop-start
        # cycling at UPDATE_PERIOD_S is what made the arm look like jello
        point.velocities = [velocities[joint] for joint in JOINT_ORDER]
        point.time_from_start = Duration(sec=0, nanosec=int(TRAJECTORY_DURATION_S * 1e9))
        msg.points = [point]
        self.trajectory_pub.publish(msg)


def main(args=None) -> None:
    """Main method"""
    rclpy.init(args=args)
    node = Arm()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
