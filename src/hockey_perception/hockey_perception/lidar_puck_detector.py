import math
import numpy as np
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


def polar_to_xy(r: float, a: float) -> Tuple[float, float]:
    return (r * math.cos(a), r * math.sin(a))


class LidarPuckDetector(Node):
    """
    Detecting a small object (the ball) as a compact cluster in a 2D LaserScan.

    A 2D lidar is a single scan plane at the lidar's mounting height. For a sphere resting on the ground,
    the "apparent" diameter seen by that scan plane can be smaller than the sphere's full diameter if
    the scan plane cuts the sphere above its equator. 

    Publishes:
      - puck_pose (PoseStamped) in the scan frame (LaserScan.header.frame_id)
    """

    def __init__(self):
        super().__init__('lidar_puck_detector')

        # Topics
        self.declare_parameter('scan_topic', '/robot1/scan')
        self.declare_parameter('puck_pose_topic', '/robot1/puck_pose_lidar')

        # Geometry / tuning
        #
        # IMPORTANT: Despite the name, `puck_diameter` is the expected apparent diameter in the 2D scan plane.
        # For a ball (sphere), that may be less than the real sphere diameter depending on lidar height.
        #
        self.declare_parameter('puck_diameter', 0.3)            # in meters 
        self.declare_parameter('diameter_tol', 0.06)            # +/- tolerance in meters
        self.declare_parameter('cluster_gap', 0.05)             # split clusters if adjacent points separated by this (m)
        self.declare_parameter('min_points', 3)                 # reject tiny clusters
        self.declare_parameter('range_min', 0.02)               # ignore anything closer than this (m)
        self.declare_parameter('range_max', 8.0)                # ignore anything beyond this (m)

        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('publish_prediction', True)      # publish prediction during short dropouts
        self.declare_parameter('max_missed_scans', 6)           # reset track after this many consecutive misses

        # Noise tuning 
        self.declare_parameter('meas_noise_std', 0.05)          # measurement std-dev on (x,y)
        self.declare_parameter('accel_noise_std', 2.0)          # process accel std-dev (white acceleration)
        self.declare_parameter('init_pos_std', 0.10)            # initial position std-dev
        self.declare_parameter('init_vel_std', 1.00)            # initial velocity std-dev

        # Data association gate
        self.declare_parameter('gate_mahalanobis_sq', 9.21)

        scan_topic = self.get_parameter('scan_topic').value
        pose_topic = self.get_parameter('puck_pose_topic').value

        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(PoseStamped, pose_topic, 10)

        # KF state: [x, y, vx, vy]
        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None
        self._last_t: Optional[float] = None
        self._missed: int = 0

        self.get_logger().info(f"Listening: {scan_topic} -> Publishing: {pose_topic}")

    def on_scan(self, msg: LaserScan) -> None:
        now = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        rmin = float(self.get_parameter('range_min').value)
        rmax = float(self.get_parameter('range_max').value)

        # Converting to XY (valid points only). Keeping invalids as NaN to force cluster breaks.
        points: List[Tuple[float, float]] = []

        a = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and (rmin <= r <= rmax):
                points.append(polar_to_xy(r, a))
            else:
                points.append((math.nan, math.nan))
            a += msg.angle_increment

        clusters = self.cluster_points(points, gap=float(self.get_parameter('cluster_gap').value))

        best = self.pick_puck_cluster(
            clusters,
            target_d=float(self.get_parameter('puck_diameter').value),
            tol=float(self.get_parameter('diameter_tol').value),
            min_pts=int(self.get_parameter('min_points').value),
        )

        if best is None:
            self._handle_measurement(None, now=now, header=msg.header)
        else:
            cx, cy = best
            self._handle_measurement((cx, cy), now=now, header=msg.header)


    def _handle_measurement(self, z_xy: Optional[Tuple[float, float]], now: float, header) -> None:
        tracking = bool(self.get_parameter('enable_tracking').value)

        if not tracking:
            if z_xy is None:
                return
            self._publish_pose(z_xy[0], z_xy[1], header)
            return

        # Computing dt
        if self._last_t is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self._last_t)
            dt = min(dt, 0.5)

        # Initialize only if tracker is not yet initialized
        if self._x is None or self._P is None:
            if z_xy is None:
                return
            self._init_track(z_xy, now)
            self._publish_pose(float(self._x[0]), float(self._x[1]), header)
            return

        # If time didn't advance, don't reinitialize and just handle update/publish safely
        if dt <= 1e-6:
            self._last_t = now  # keep timestamps moving forward if possible
            if z_xy is None:
                if bool(self.get_parameter('publish_prediction').value):
                    self._publish_pose(float(self._x[0]), float(self._x[1]), header)
                return

            z = np.array([[z_xy[0]], [z_xy[1]]], dtype=float)
            if self._passes_gate(z):
                self._update(z)
                self._missed = 0
            else:
                self._missed += 1
            self._publish_pose(float(self._x[0]), float(self._x[1]), header)
            return

        self._predict(dt)
        self._last_t = now

        if z_xy is None:
            self._missed += 1
            if self._missed > int(self.get_parameter('max_missed_scans').value):
                self._reset_track()
                return

            if bool(self.get_parameter('publish_prediction').value):
                self._publish_pose(float(self._x[0]), float(self._x[1]), header)
            return

        # Updating measurement with gating
        z = np.array([[z_xy[0]], [z_xy[1]]], dtype=float)
        if self._passes_gate(z):
            self._update(z)
            self._missed = 0
            self._publish_pose(float(self._x[0]), float(self._x[1]), header)
        else:
            self._missed += 1
            if self._missed > int(self.get_parameter('max_missed_scans').value):
                self._reset_track()
                return
            if bool(self.get_parameter('publish_prediction').value):
                self._publish_pose(float(self._x[0]), float(self._x[1]), header)

    def _init_track(self, z_xy: Tuple[float, float], now: float) -> None:
        init_pos_std = float(self.get_parameter('init_pos_std').value)
        init_vel_std = float(self.get_parameter('init_vel_std').value)

        self._x = np.array([[z_xy[0]], [z_xy[1]], [0.0], [0.0]], dtype=float)
        self._P = np.diag([
            init_pos_std ** 2,
            init_pos_std ** 2,
            init_vel_std ** 2,
            init_vel_std ** 2,
        ]).astype(float)

        self._last_t = now
        self._missed = 0

    def _reset_track(self) -> None:
        self._x = None
        self._P = None
        self._last_t = None
        self._missed = 0

    def _predict(self, dt: float) -> None:
        assert self._x is not None and self._P is not None

        # State transition
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        # Processing noise
        a_std = float(self.get_parameter('accel_noise_std').value)
        q = a_std ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = q * np.array([
            [dt4 / 4.0, 0.0,       dt3 / 2.0, 0.0      ],
            [0.0,       dt4 / 4.0, 0.0,       dt3 / 2.0],
            [dt3 / 2.0, 0.0,       dt2,       0.0      ],
            [0.0,       dt3 / 2.0, 0.0,       dt2      ],
        ], dtype=float)

        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def _passes_gate(self, z: np.ndarray) -> bool:
        assert self._x is not None and self._P is not None

        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)
        r_std = float(self.get_parameter('meas_noise_std').value)
        R = (r_std ** 2) * np.eye(2, dtype=float)

        y = z - (H @ self._x)
        S = H @ self._P @ H.T + R
        try:
            d2 = float((y.T @ np.linalg.inv(S) @ y)[0, 0])
        except np.linalg.LinAlgError:
            return False

        gate = float(self.get_parameter('gate_mahalanobis_sq').value)
        return d2 <= gate

    def _update(self, z: np.ndarray) -> None:
        assert self._x is not None and self._P is not None

        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)
        r_std = float(self.get_parameter('meas_noise_std').value)
        R = (r_std ** 2) * np.eye(2, dtype=float)

        y = z - (H @ self._x)
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)
        I = np.eye(4, dtype=float)

        self._x = self._x + (K @ y)
        self._P = (I - K @ H) @ self._P

    def _publish_pose(self, x: float, y: float, header) -> None:
        out = PoseStamped()
        out.header = header
        out.pose.position.x = float(x)
        out.pose.position.y = float(y)
        out.pose.position.z = 0.0
        out.pose.orientation.w = 1.0
        self.pub.publish(out)

    @staticmethod
    def cluster_points(points: List[Tuple[float, float]], gap: float) -> List[List[Tuple[float, float]]]:
        clusters: List[List[Tuple[float, float]]] = []
        cur: List[Tuple[float, float]] = []
        prev: Optional[Tuple[float, float]] = None

        for p in points:
            if not (math.isfinite(p[0]) and math.isfinite(p[1])):
                if cur:
                    clusters.append(cur)
                    cur = []
                prev = None
                continue

            if prev is None:
                cur.append(p)
            else:
                dx = p[0] - prev[0]
                dy = p[1] - prev[1]
                if math.hypot(dx, dy) > gap:
                    if cur:
                        clusters.append(cur)
                    cur = [p]
                else:
                    cur.append(p)

            prev = p

        if cur:
            clusters.append(cur)

        return clusters

    @staticmethod
    def _cluster_width_bbox_diag(c: List[Tuple[float, float]]) -> float:
        # Computing a single scalar 'width'
        xs = [p[0] for p in c]
        ys = [p[1] for p in c]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    @staticmethod
    def pick_puck_cluster(
        clusters: List[List[Tuple[float, float]]],
        target_d: float,
        tol: float,
        min_pts: int,
    ) -> Optional[Tuple[float, float]]:
        
        # Choosing the best cluster matching the expected apparent diameter, and returning centroid (x,y).
        
        best_score = float('inf')
        best_centroid: Optional[Tuple[float, float]] = None

        for c in clusters:
            if len(c) < min_pts:
                continue

            width = LidarPuckDetector._cluster_width_bbox_diag(c)

            # Size gate
            size_err = abs(width - target_d)
            if size_err > tol:
                continue

            # Centroid
            sx = sum(p[0] for p in c)
            sy = sum(p[1] for p in c)
            cx = sx / len(c)
            cy = sy / len(c)

            # Score: prefer closer-to-expected size, and slightly prefer nearer objects
            range_penalty = math.hypot(cx, cy) * 0.05
            score = size_err + range_penalty

            if score < best_score:
                best_score = score
                best_centroid = (cx, cy)

        return best_centroid   


def main():
    rclpy.init()
    node = LidarPuckDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
