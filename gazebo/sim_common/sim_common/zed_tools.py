"""
Numpy helpers for the simulated ZED 2i (see zed_camera.py)
"""

import math

import numpy as np


class PinholeCamera:
    """Rectified pinhole intrinsics plus per-pixel normalised ray coordinates"""

    def __init__(self, width: int, height: int, fx: float, fy: float, cx: float, cy: float):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        # optical-frame ray (x right, y down) through each pixel centre, at z = 1
        self.ray_x = np.broadcast_to((np.arange(width) - cx) / fx, (height, width))
        self.ray_y = np.broadcast_to(((np.arange(height) - cy) / fy)[:, None], (height, width))

    @classmethod
    def from_camera_info(cls, info) -> "PinholeCamera":
        return cls(info.width, info.height, info.k[0], info.k[4], info.k[2], info.k[5])


def lidar_lookup(camera: PinholeCamera, lidar_width: int, lidar_height: int) -> np.ndarray:
    """
    Flat index into a gz gpu_lidar organised cloud for each pixel of `camera`, picking
    the ray nearest the pixel's own ray. The lidar must sit on the camera's frame and
    span its frustum: azimuth +-atan((w/2)/fx), elevation +-atan((h/2)/fy), with row 0
    at the lowest elevation and column 0 at the most negative azimuth (image right)
    """
    az_max = math.atan((camera.width / 2) / camera.fx)
    el_max = math.atan((camera.height / 2) / camera.fy)

    # pixel ray in the lidar's body frame (x forward, y left, z up) is (1, -ray_x, -ray_y)
    azimuth = np.arctan(-camera.ray_x)
    elevation = np.arctan2(-camera.ray_y, np.sqrt(1.0 + camera.ray_x**2))

    col = np.rint((azimuth + az_max) / (2 * az_max) * (lidar_width - 1)).astype(np.int64)
    row = np.rint((elevation + el_max) / (2 * el_max) * (lidar_height - 1)).astype(np.int64)
    np.clip(col, 0, lidar_width - 1, out=col)
    np.clip(row, 0, lidar_height - 1, out=row)
    return (row * lidar_width + col).ravel()


class SmoothNoise:
    """
    Unit-variance gaussian field, drawn on a coarse grid and bilinearly upsampled so
    errors come in smooth patches like neural stereo depth rather than per-pixel speckle
    """

    def __init__(self, height: int, width: int, cell: int, rng: np.random.Generator):
        self.rng = rng
        self.row_weights = self._interpolation(height, cell)
        self.col_weights = self._interpolation(width, cell)
        # each pixel mixes coarse samples with weights wy*wx; rescale back to unit variance
        self.stddev = np.outer(
            np.sqrt((self.row_weights**2).sum(axis=1)), np.sqrt((self.col_weights**2).sum(axis=1))
        )

    @staticmethod
    def _interpolation(size: int, cell: int) -> np.ndarray:
        """(size x coarse) matrix linearly interpolating coarse samples spaced `cell` apart"""
        coarse = size // cell + 2
        pos = np.arange(size) / cell
        lower = np.floor(pos).astype(int)
        frac = pos - lower
        weights = np.zeros((size, coarse))
        weights[np.arange(size), lower] = 1.0 - frac
        weights[np.arange(size), lower + 1] = frac
        return weights

    def sample(self) -> np.ndarray:
        coarse = self.rng.standard_normal((self.row_weights.shape[1], self.col_weights.shape[1]))
        return (self.row_weights @ coarse @ self.col_weights.T) / self.stddev


def stereo_depth(
    true_depth: np.ndarray,
    noise: np.ndarray,
    fx: float,
    baseline: float,
    disparity_stddev: float,
    min_depth: float,
    max_depth: float,
) -> np.ndarray:
    """
    Add stereo matching error and apply the ZED's invalid-depth conventions.
    A disparity error of sigma_d px gives a depth error of z^2 * sigma_d / (fx * B).
    Too close is -inf, too far (or no return) is +inf
    """
    with np.errstate(invalid="ignore", over="ignore"):
        depth = true_depth + true_depth**2 * (disparity_stddev / (fx * baseline)) * noise
        depth = depth.astype(np.float32)
        depth[depth < min_depth] = -np.inf
        depth[depth > max_depth] = np.inf
    # out-of-range rays are +-inf in the true depth, which the noise term turned to NaN
    depth[np.isposinf(true_depth)] = np.inf
    depth[np.isneginf(true_depth)] = -np.inf
    return depth


def rgb_to_bgra(rgb: np.ndarray) -> np.ndarray:
    bgra = np.empty(rgb.shape[:2] + (4,), dtype=np.uint8)
    bgra[..., 0] = rgb[..., 2]
    bgra[..., 1] = rgb[..., 1]
    bgra[..., 2] = rgb[..., 0]
    bgra[..., 3] = 255
    return bgra


def organised_cloud(depth: np.ndarray, bgra: np.ndarray, camera: PinholeCamera) -> np.ndarray:
    """
    (h, w, 4) float32 x/y/z/rgb points in the non-optical camera frame (x forward,
    y left, z up) like the ZED's cloud_registered, NaN where depth is invalid. The rgb
    float carries the BGRA bytes, which reads back as PCL's packed 0x00RRGGBB
    """
    valid = np.isfinite(depth)
    z = np.where(valid, depth, np.nan).astype(np.float32)

    cloud = np.empty(depth.shape + (4,), dtype=np.float32)
    cloud[..., 0] = z
    cloud[..., 1] = -camera.ray_x * z
    cloud[..., 2] = -camera.ray_y * z
    cloud[..., 3] = bgra.view(np.float32)[..., 0]
    return cloud


# ----------------------------------------------
# Rigid transforms (4x4 homogeneous matrices)
# ----------------------------------------------


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(rot: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) for a rotation matrix"""
    trace = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return (
            (rot[2, 1] - rot[1, 2]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
            (rot[1, 0] - rot[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax(np.diag(rot)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + rot[i, i] - rot[j, j] - rot[k, k])
    q = [0.0, 0.0, 0.0, (rot[k, j] - rot[j, k]) / s]
    q[i] = 0.25 * s
    q[j] = (rot[j, i] + rot[i, j]) / s
    q[k] = (rot[k, i] + rot[i, k]) / s
    return q[0], q[1], q[2], q[3]


def pose_matrix(position, orientation) -> np.ndarray:
    """4x4 transform from geometry_msgs Point/Vector3 and Quaternion"""
    tf = np.eye(4)
    tf[:3, :3] = quaternion_to_matrix(orientation.x, orientation.y, orientation.z, orientation.w)
    tf[:3, 3] = (position.x, position.y, position.z)
    return tf


def yaw_matrix(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    tf = np.eye(4)
    tf[:2, :2] = ((c, -s), (s, c))
    return tf


def yaw_of(tf: np.ndarray) -> float:
    return math.atan2(tf[1, 0], tf[0, 0])


def invert(tf: np.ndarray) -> np.ndarray:
    inv = np.eye(4)
    inv[:3, :3] = tf[:3, :3].T
    inv[:3, 3] = -tf[:3, :3].T @ tf[:3, 3]
    return inv
