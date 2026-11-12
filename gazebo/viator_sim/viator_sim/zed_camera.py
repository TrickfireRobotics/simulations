"""
Sim equivalent of the real rover's StereoLabs ZED 2i (zed-ros2-wrapper v5.x)

Takes the chassis' noise-free gz camera, depth rays, IMU and ground-truth pose, adds
ZED-like errors and republishes them as `zed_node` on the wrapper's topics, with its
encodings (bgra8 colour, 32FC1 metres depth, x/y/z/rgb cloud), frames and
invalid-depth conventions. Like the wrapper, each output is only computed while
something subscribes to it.

Intentional differences from the real camera: images run at the sim's render rate
(the wrapper grabs at 60 Hz), positional tracking has no loop closure, and the
odom -> camera_link TF is not published (the sim's robot TF is authoritative).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from viator_sim import zed_tools as zt

# ground truth from viator_description/urdf/viator_zed.urdf.xacro, via ros_gz_bridge
IMAGE_TOPIC = "/viator/zed/image"
CAMERA_INFO_TOPIC = "/viator/zed/camera_info"
DEPTH_RAYS_TOPIC = "/viator/zed/depth_rays/points"
IMU_TOPIC = "/viator/zed/imu"
GROUND_TRUTH_TOPIC = "/viator/ground_truth/odometry"
BASE_FRAME = "chassis_base_link"

# the wrapper's QOS_QUEUE_SIZE (zed_components sl_types.hpp): keep-last 10, reliable.
# Large frames need the history to resend fragments a reader missed
QOS = QoSProfile(depth=10)

CLOUD_FIELDS = [
    PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
    for i, name in enumerate(("x", "y", "z", "rgb"))
]

# cloud pixel stride for the wrapper's depth.point_cloud_res: COMPACT full, REDUCED half
POINT_CLOUD_STRIDES = {"COMPACT": 1, "REDUCED": 2}

# coarse grid spacing (px) of the smooth depth noise
DEPTH_NOISE_CELL = 8
# roll/pitch are IMU-fused on the real camera, so they stay tight
TILT_STDDEV = math.radians(0.5)
# distance (m) over which the odometry drift biases wander
DRIFT_CORRELATION_DISTANCE = 20.0
# rover counts as at rest after spawning once below this speed (m/s) for this long (s)
SETTLED_SPEED = 0.01
SETTLE_TIME = 0.5


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class RateLimiter:
    """
    Passes stamps at `rate` Hz on average, even when the input isn't a multiple of
    it (10 Hz out of 15 Hz frames alternates 1- and 2-frame gaps). 0 = no limit
    """

    # slack for stamps quantised to the physics step
    TOLERANCE = 0.002

    def __init__(self, rate: float):
        self.period = 1.0 / rate if rate > 0.0 else 0.0
        self.next = -math.inf

    def ready(self, stamp: float) -> bool:
        if stamp + self.TOLERANCE < self.next:
            return False
        # stay on schedule, but don't burst to catch up after a gap
        self.next = (
            self.next + self.period if stamp - self.next < self.period else stamp + self.period
        )
        return True


def _diagonal(values) -> list[float]:
    size = len(values)
    out = [0.0] * size * size
    for i, value in enumerate(values):
        out[i * (size + 1)] = value
    return out


def _pose(tf: np.ndarray) -> Pose:
    x, y, z, w = zt.matrix_to_quaternion(tf[:3, :3])
    return Pose(
        position=Point(x=tf[0, 3], y=tf[1, 3], z=tf[2, 3]),
        orientation=Quaternion(x=x, y=y, z=z, w=w),
    )


class ZedCamera(Node):
    """Main class"""

    def __init__(self):
        # launched in namespace `zed`, so `~/...` resolves to /zed/zed_node/...
        super().__init__("zed_node")

        camera_name = self.declare_parameter("camera_name", "zed").value
        self.pub_frame_rate = self.declare_parameter("pub_frame_rate", 15.0).value
        self.point_cloud_freq = self.declare_parameter("point_cloud_freq", 10.0).value
        point_cloud_res = self.declare_parameter("point_cloud_res", "COMPACT").value
        if point_cloud_res not in POINT_CLOUD_STRIDES:
            raise ValueError(f"point_cloud_res must be COMPACT or REDUCED, not '{point_cloud_res}'")
        self.cloud_stride = POINT_CLOUD_STRIDES[point_cloud_res]
        self.min_depth = self.declare_parameter("min_depth", 0.3).value
        self.max_depth = self.declare_parameter("max_depth", 15.0).value
        self.depth_noise_px = self.declare_parameter("depth_noise_px", 0.15).value
        self.baseline = self.declare_parameter("baseline", 0.12).value
        gyro_stddev = self.declare_parameter("imu_gyro_stddev", 0.0017).value
        accel_stddev = self.declare_parameter("imu_accel_stddev", 0.0083).value
        self.drift_ratio = self.declare_parameter("odom_drift_ratio", 0.01).value
        self.yaw_drift = math.radians(self.declare_parameter("odom_yaw_drift_deg_per_m", 0.5).value)
        path_pub_rate = self.declare_parameter("path_pub_rate", 2.0).value

        # frame names match the wrapper's, built from camera_name
        self.optical_frame = f"{camera_name}_left_camera_frame_optical"
        self.cloud_frame = f"{camera_name}_left_camera_frame"
        self.imu_frame = f"{camera_name}_imu_link"
        self.camera_link = f"{camera_name}_camera_link"

        # video, IMU and odometry each run in their own callback group on a
        # multi-threaded executor, so heavy frames can't starve the 100 Hz IMU
        self._init_video()
        self._init_imu(gyro_stddev, accel_stddev)
        self._init_odometry(path_pub_rate)

        self.get_logger().info(f"Emulating ZED 2i '{camera_name}'")

    # ----------------------------------------------
    # RGB, depth and point cloud
    # ----------------------------------------------

    def _init_video(self) -> None:
        self.camera: zt.PinholeCamera | None = None
        self.noise: zt.SmoothNoise | None = None
        self.lookup: np.ndarray | None = None
        self.frame_limiter = RateLimiter(self.pub_frame_rate)
        self.cloud_limiter = RateLimiter(self.point_cloud_freq)

        self.rgb_pub = self.create_publisher(Image, "~/rgb/color/rect/image", QOS)
        self.rgb_info_pub = self.create_publisher(CameraInfo, "~/rgb/color/rect/camera_info", QOS)
        self.depth_pub = self.create_publisher(Image, "~/depth/depth_registered", QOS)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "~/depth/depth_registered/camera_info", QOS
        )
        self.cloud_pub = self.create_publisher(PointCloud2, "~/point_cloud/cloud_registered", QOS)

        # message_filters can't take a callback group, so video stays in the node's
        # default (mutually exclusive) group; IMU and odometry get their own
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, QOS)
        sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Image, IMAGE_TOPIC, qos_profile=QoSProfile(depth=2)),
                Subscriber(self, PointCloud2, DEPTH_RAYS_TOPIC, qos_profile=QoSProfile(depth=2)),
            ],
            queue_size=4,
            slop=0.01,
        )
        sync.registerCallback(self._on_frame)

    def _on_camera_info(self, info: CameraInfo) -> None:
        if self.camera is not None:
            return
        self.camera = zt.PinholeCamera.from_camera_info(info)
        self.noise = zt.SmoothNoise(
            info.height, info.width, DEPTH_NOISE_CELL, np.random.default_rng()
        )

    def _on_frame(self, image: Image, rays: PointCloud2) -> None:
        if self.camera is None:
            return

        stamp = _seconds(image.header.stamp)
        if not self.frame_limiter.ready(stamp):
            return

        want_rgb = self._subscribed(self.rgb_pub, self.rgb_info_pub)
        want_depth = self._subscribed(self.depth_pub, self.depth_info_pub)
        want_cloud = self._subscribed(self.cloud_pub) and self.cloud_limiter.ready(stamp)

        header = Header(stamp=image.header.stamp, frame_id=self.optical_frame)

        bgra = None
        if want_rgb or want_cloud:
            rgb = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
            bgra = zt.rgb_to_bgra(rgb)

        if want_rgb:
            self.rgb_pub.publish(self._image(header, bgra, "bgra8"))
            self.rgb_info_pub.publish(self._camera_info(header))

        if not (want_depth or want_cloud):
            return

        depth = self._depth(rays)

        if want_depth:
            self.depth_pub.publish(self._image(header, depth, "32FC1"))
            self.depth_info_pub.publish(self._camera_info(header))

        if want_cloud:
            stride = self.cloud_stride
            points = zt.organised_cloud(depth, bgra, self.camera)[::stride, ::stride]
            cloud = PointCloud2(
                header=Header(stamp=image.header.stamp, frame_id=self.cloud_frame),
                height=points.shape[0],
                width=points.shape[1],
                fields=CLOUD_FIELDS,
                is_bigendian=False,
                point_step=16,
                row_step=16 * points.shape[1],
                is_dense=False,
            )
            cloud.data = points.tobytes()
            self.cloud_pub.publish(cloud)

    def _depth(self, rays: PointCloud2) -> np.ndarray:
        """Resample the gz depth rays into a pinhole depth image, then add stereo error"""
        if self.lookup is None:
            self.lookup = zt.lidar_lookup(self.camera, rays.width, rays.height)

        # a ray's forward (x) component is the pixel's depth along the optical axis
        x_offset = next(field.offset for field in rays.fields if field.name == "x")
        points = np.frombuffer(rays.data, np.float32).reshape(-1, rays.point_step // 4)
        true_depth = points[:, x_offset // 4][self.lookup].reshape(
            self.camera.height, self.camera.width
        )

        return zt.stereo_depth(
            true_depth,
            self.noise.sample(),
            self.camera.fx,
            self.baseline,
            self.depth_noise_px,
            self.min_depth,
            self.max_depth,
        )

    def _camera_info(self, header: Header) -> CameraInfo:
        """Rectified intrinsics, as the wrapper publishes for its rect images"""
        cam = self.camera
        return CameraInfo(
            header=header,
            height=cam.height,
            width=cam.width,
            distortion_model="plumb_bob",
            d=[0.0] * 5,
            k=[cam.fx, 0.0, cam.cx, 0.0, cam.fy, cam.cy, 0.0, 0.0, 1.0],
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[cam.fx, 0.0, cam.cx, 0.0, 0.0, cam.fy, cam.cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        )

    @staticmethod
    def _image(header: Header, array: np.ndarray, encoding: str) -> Image:
        msg = Image(
            header=header,
            height=array.shape[0],
            width=array.shape[1],
            encoding=encoding,
            is_bigendian=0,
            step=array.shape[1] * array.itemsize * (array.shape[2] if array.ndim == 3 else 1),
        )
        msg.data = array.tobytes()
        return msg

    @staticmethod
    def _subscribed(*publishers) -> bool:
        return any(pub.get_subscription_count() > 0 for pub in publishers)

    # ----------------------------------------------
    # IMU
    # ----------------------------------------------

    def _init_imu(self, gyro_stddev: float, accel_stddev: float) -> None:
        self.imu_start_yaw: float | None = None
        self.orientation_covariance = _diagonal([TILT_STDDEV**2] * 3)
        self.gyro_covariance = _diagonal([gyro_stddev**2] * 3)
        self.accel_covariance = _diagonal([accel_stddev**2] * 3)

        self.imu_pub = self.create_publisher(Imu, "~/imu/data", QOS)
        self.imu_raw_pub = self.create_publisher(Imu, "~/imu/data_raw", QOS)
        self.create_subscription(
            Imu,
            IMU_TOPIC,
            self._on_imu,
            QoSProfile(depth=10),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _on_imu(self, truth: Imu) -> None:
        header = Header(stamp=truth.header.stamp, frame_id=self.imu_frame)

        if self._subscribed(self.imu_pub):
            # gravity-aligned roll/pitch; yaw is relative to where the camera started
            q = truth.orientation
            rot = zt.quaternion_to_matrix(q.x, q.y, q.z, q.w)
            if self.imu_start_yaw is None:
                self.imu_start_yaw = math.atan2(rot[1, 0], rot[0, 0])
            x, y, z, w = zt.matrix_to_quaternion(zt.yaw_matrix(-self.imu_start_yaw)[:3, :3] @ rot)

            self.imu_pub.publish(
                Imu(
                    header=header,
                    orientation=Quaternion(x=x, y=y, z=z, w=w),
                    orientation_covariance=self.orientation_covariance,
                    angular_velocity=truth.angular_velocity,
                    angular_velocity_covariance=self.gyro_covariance,
                    linear_acceleration=truth.linear_acceleration,
                    linear_acceleration_covariance=self.accel_covariance,
                )
            )

        if self._subscribed(self.imu_raw_pub):
            # no orientation estimate on the raw topic
            raw_orientation_covariance = [0.0] * 9
            raw_orientation_covariance[0] = -1.0
            self.imu_raw_pub.publish(
                Imu(
                    header=header,
                    orientation_covariance=raw_orientation_covariance,
                    angular_velocity=truth.angular_velocity,
                    angular_velocity_covariance=self.gyro_covariance,
                    linear_acceleration=truth.linear_acceleration,
                    linear_acceleration_covariance=self.accel_covariance,
                )
            )

    # ----------------------------------------------
    # Visual odometry (positional tracking)
    # ----------------------------------------------

    def _init_odometry(self, path_pub_rate: float) -> None:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_to_camera: np.ndarray | None = None
        self.settled_since: float | None = None
        self.odom_limiter = RateLimiter(self.pub_frame_rate)
        self.prev_world_camera: np.ndarray | None = None
        self.odom_camera: np.ndarray | None = None
        self.distance = 0.0
        self.last_odom = -math.inf
        self.last_odom_stamp = None

        # drift accrues as slowly varying per-metre biases, so error grows with
        # distance like real visual odometry instead of averaging out step to step
        # numpy generators aren't thread-safe, so odometry keeps its own
        self.rng = np.random.default_rng()
        self.drift_bias = self.rng.normal(0.0, self.drift_ratio, 3)
        self.yaw_bias = self.rng.normal(0.0, self.yaw_drift)

        self.odom_pub = self.create_publisher(Odometry, "~/odom", QOS)
        self.pose_pub = self.create_publisher(PoseStamped, "~/pose", QOS)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, "~/pose_with_covariance", QOS
        )
        self.path_odom_pub = self.create_publisher(Path, "~/path_odom", QOS)
        self.path_map_pub = self.create_publisher(Path, "~/path_map", QOS)
        self.path_odom = Path(header=Header(frame_id="odom"))
        self.path_map = Path(header=Header(frame_id="map"))

        group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Odometry,
            GROUND_TRUTH_TOPIC,
            self._on_ground_truth,
            QoSProfile(depth=10),
            callback_group=group,
        )
        if path_pub_rate > 0.0:
            self.create_timer(1.0 / path_pub_rate, self._publish_paths, callback_group=group)

    def _on_ground_truth(self, truth: Odometry) -> None:
        stamp = _seconds(truth.header.stamp)

        if self.prev_world_camera is None and not self._ready_to_track(truth, stamp):
            return
        if not self.odom_limiter.ready(stamp):
            return

        world_camera = zt.pose_matrix(truth.pose.pose.position, truth.pose.pose.orientation)
        world_camera = world_camera @ self.base_to_camera

        if self.prev_world_camera is None:
            # odom starts at the camera's initial position, gravity-aligned with zero yaw
            start = zt.yaw_matrix(zt.yaw_of(world_camera))
            start[:3, 3] = world_camera[:3, 3]
            self.odom_camera = zt.invert(start) @ world_camera
            linear = angular = np.zeros(3)
        else:
            dt = stamp - self.last_odom
            step = self._drifted(zt.invert(self.prev_world_camera) @ world_camera)
            self.odom_camera = self.odom_camera @ step
            # twist in the child (camera_link) frame, like the wrapper
            linear = step[:3, 3] / dt
            skew = (step[:3, :3] - step[:3, :3].T) / 2
            angular = np.array((skew[2, 1], skew[0, 2], skew[1, 0])) / dt

        self.prev_world_camera = world_camera
        self.last_odom = stamp
        self.last_odom_stamp = truth.header.stamp
        self._publish_odometry(truth.header.stamp, linear, angular)

    def _ready_to_track(self, truth: Odometry, stamp: float) -> bool:
        """
        Tracking starts once the camera's mount transform is known and the rover has
        settled after spawning, as a real ZED starts at rest rather than mid-drop
        """
        if self.base_to_camera is None:
            try:
                tf = self.tf_buffer.lookup_transform(BASE_FRAME, self.camera_link, Time())
            except TransformException:
                self.get_logger().warn(
                    f"Waiting for the {BASE_FRAME} -> {self.camera_link} transform",
                    throttle_duration_sec=5.0,
                )
                return False
            self.base_to_camera = zt.pose_matrix(tf.transform.translation, tf.transform.rotation)

        v = truth.twist.twist.linear
        if math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) > SETTLED_SPEED:
            self.settled_since = None
            return False
        if self.settled_since is None:
            self.settled_since = stamp
        return stamp - self.settled_since >= SETTLE_TIME

    def _drifted(self, step: np.ndarray) -> np.ndarray:
        """True camera motion for one step, corrupted by the current drift biases"""
        distance = float(np.linalg.norm(step[:3, 3]))
        self.distance += distance

        # first-order Gauss-Markov over distance travelled
        decay = math.exp(-distance / DRIFT_CORRELATION_DISTANCE)
        innovation = math.sqrt(1.0 - decay**2)
        self.drift_bias = decay * self.drift_bias + innovation * self.rng.normal(
            0.0, self.drift_ratio, 3
        )
        self.yaw_bias = decay * self.yaw_bias + innovation * self.rng.normal(0.0, self.yaw_drift)

        noisy = step.copy()
        noisy[:3, 3] += self.drift_bias * distance
        return noisy @ zt.yaw_matrix(self.yaw_bias * distance)

    def _publish_odometry(self, stamp, linear: np.ndarray, angular: np.ndarray) -> None:
        pose = _pose(self.odom_camera)
        position_var = (self.drift_ratio * self.distance) ** 2 + 1e-6
        yaw_var = (self.yaw_drift * self.distance) ** 2 + 1e-6
        pose_covariance = _diagonal([position_var] * 3 + [TILT_STDDEV**2] * 2 + [yaw_var])

        odom = Odometry(
            header=Header(stamp=stamp, frame_id="odom"), child_frame_id=self.camera_link
        )
        odom.pose.pose = pose
        odom.pose.covariance = pose_covariance
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = linear
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = angular
        odom.twist.covariance = _diagonal([1e-4] * 6)
        self.odom_pub.publish(odom)

        # no loop closure in the sim, so map == odom
        map_header = Header(stamp=stamp, frame_id="map")
        self.pose_pub.publish(PoseStamped(header=map_header, pose=pose))
        pose_cov = PoseWithCovarianceStamped(header=map_header)
        pose_cov.pose.pose = pose
        pose_cov.pose.covariance = pose_covariance
        self.pose_cov_pub.publish(pose_cov)

    def _publish_paths(self) -> None:
        if self.odom_camera is None:
            return
        pose = _pose(self.odom_camera)
        for path, pub in ((self.path_odom, self.path_odom_pub), (self.path_map, self.path_map_pub)):
            path.header.stamp = self.last_odom_stamp
            path.poses.append(
                PoseStamped(
                    header=Header(stamp=self.last_odom_stamp, frame_id=path.header.frame_id),
                    pose=pose,
                )
            )
            pub.publish(path)


def main(args=None):
    """Main method"""
    rclpy.init(args=args)
    node = ZedCamera()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
