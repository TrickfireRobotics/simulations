"""
Sim equivalent of the real rover's u-blox ZED-F9P driver (KumarRobotics ublox_gps)

Takes the noise-free gz NavSat reading, adds receiver error and republishes it as
`ublox_gps_node` on the same `~/fix` and `~/fix_velocity` topics, with the same
status/covariance conventions as ublox_gps/ublox_firmware7plus.hpp.

The real driver stamps messages with GNSS UTC time from NAV-PVT; here the stamp is
copied from the sim reading so it lines up with the rest of the sim-time graph.
"""

import math
import random

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

# WGS84 ellipsoid
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

# ZED-F9P datasheet accuracies, as 1-sigma per axis.
# correlation_time > 0 makes the position error a first-order Gauss-Markov process
# so it wanders like a real receiver's instead of jumping independently every fix
FIX_MODES = {
    "rtk_fixed": {
        "horizontal_stddev": 0.01,  # m
        "vertical_stddev": 0.01,  # m
        "velocity_stddev": 0.05,  # m/s
        "correlation_time": 0.0,  # s
        # ublox_gps reports GBAS_FIX when the carrier phase solution is fixed
        "status": NavSatStatus.STATUS_GBAS_FIX,
    },
    "standalone": {
        "horizontal_stddev": 1.5,
        "vertical_stddev": 2.5,
        "velocity_stddev": 0.05,
        "correlation_time": 30.0,
        "status": NavSatStatus.STATUS_FIX,
    },
}

# ublox_gps derives this from its gnss.* params; the config the real rover loads
# enables GPS + GLONASS
SERVICE = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS


def _earth_radii(lat_rad: float) -> tuple[float, float]:
    """Meridian and prime vertical radii of curvature at `lat_rad`"""
    sin2 = math.sin(lat_rad) ** 2
    meridian = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * sin2) ** 1.5
    prime_vertical = WGS84_A / math.sqrt(1 - WGS84_E2 * sin2)
    return meridian, prime_vertical


def _enu_delta(prev: NavSatFix, curr: NavSatFix) -> tuple[float, float, float]:
    """Displacement in metres (east, north, up) between two fixes"""
    lat = math.radians(curr.latitude)
    meridian, prime_vertical = _earth_radii(lat)
    east = math.radians(curr.longitude - prev.longitude) * (
        prime_vertical + curr.altitude
    ) * math.cos(lat)
    north = math.radians(curr.latitude - prev.latitude) * (meridian + curr.altitude)
    return east, north, curr.altitude - prev.altitude


def _stamp_seconds(msg: NavSatFix) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class UbloxGps(Node):
    """Main class"""

    def __init__(self):
        # node name matches the real driver so `~/fix` resolves to /ublox_gps_node/fix
        super().__init__("ublox_gps_node")

        self.frame_id = self.declare_parameter("frame_id", "gps").value
        fix_mode = self.declare_parameter("fix_mode", "rtk_fixed").value
        input_topic = self.declare_parameter("input_topic", "/chassis/gps/navsat").value

        if fix_mode not in FIX_MODES:
            raise ValueError(f"fix_mode must be one of {list(FIX_MODES)}, got '{fix_mode}'")
        self.mode = FIX_MODES[fix_mode]

        self.error_enu = [0.0, 0.0, 0.0]
        self.prev_truth: NavSatFix | None = None

        self.fix_pub = self.create_publisher(NavSatFix, "~/fix", 1)
        self.velocity_pub = self.create_publisher(
            TwistWithCovarianceStamped, "~/fix_velocity", 1
        )

        self.create_subscription(NavSatFix, input_topic, self._on_truth, 10)

        self.get_logger().info(f"Emulating ZED-F9P in '{fix_mode}' mode from {input_topic}")

    def _on_truth(self, truth: NavSatFix) -> None:
        dt = _stamp_seconds(truth) - _stamp_seconds(self.prev_truth) if self.prev_truth else 0.0

        self._update_error(dt)
        self._publish_fix(truth)
        if self.prev_truth is not None and dt > 0.0:
            self._publish_velocity(truth, dt)

        self.prev_truth = truth

    def _update_error(self, dt: float) -> None:
        """Step the per-axis ENU position error, keeping its stationary stddev"""
        tau = self.mode["correlation_time"]
        decay = math.exp(-dt / tau) if tau > 0.0 and self.prev_truth is not None else 0.0
        innovation = math.sqrt(1.0 - decay**2)

        stddevs = (
            self.mode["horizontal_stddev"],
            self.mode["horizontal_stddev"],
            self.mode["vertical_stddev"],
        )
        self.error_enu = [
            decay * err + innovation * random.gauss(0.0, stddev)
            for err, stddev in zip(self.error_enu, stddevs)
        ]

    def _publish_fix(self, truth: NavSatFix) -> None:
        lat = math.radians(truth.latitude)
        meridian, prime_vertical = _earth_radii(lat)
        east, north, up = self.error_enu

        fix = NavSatFix()
        fix.header.stamp = truth.header.stamp
        fix.header.frame_id = self.frame_id
        fix.status.status = self.mode["status"]
        fix.status.service = SERVICE
        fix.latitude = truth.latitude + math.degrees(north / (meridian + truth.altitude))
        fix.longitude = truth.longitude + math.degrees(
            east / ((prime_vertical + truth.altitude) * math.cos(lat))
        )
        fix.altitude = truth.altitude + up

        var_h = self.mode["horizontal_stddev"] ** 2
        var_v = self.mode["vertical_stddev"] ** 2
        fix.position_covariance = [var_h, 0.0, 0.0, 0.0, var_h, 0.0, 0.0, 0.0, var_v]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.fix_pub.publish(fix)

    def _publish_velocity(self, truth: NavSatFix, dt: float) -> None:
        """ENU velocity from consecutive noise-free fixes, plus speed noise"""
        stddev = self.mode["velocity_stddev"]
        east, north, up = (
            d / dt + random.gauss(0.0, stddev) for d in _enu_delta(self.prev_truth, truth)
        )

        velocity = TwistWithCovarianceStamped()
        velocity.header.stamp = truth.header.stamp
        velocity.header.frame_id = self.frame_id
        velocity.twist.twist.linear.x = east
        velocity.twist.twist.linear.y = north
        velocity.twist.twist.linear.z = up

        # ublox_gps only knows linear velocity accuracy; angular is marked unsupported
        covariance = [0.0] * 36
        for i in range(3):
            covariance[i * 7] = stddev**2
        for i in range(3, 6):
            covariance[i * 7] = -1.0
        velocity.twist.covariance = covariance

        self.velocity_pub.publish(velocity)


def main(args=None):
    """Main method"""
    rclpy.init(args=args)
    node = UbloxGps()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
