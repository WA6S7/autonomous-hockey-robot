import math
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

        scan_topic = self.get_parameter('scan_topic').value
        pose_topic = self.get_parameter('puck_pose_topic').value

        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(PoseStamped, pose_topic, 10)

        self.get_logger().info(f"Listening: {scan_topic} -> Publishing: {pose_topic}")

    def on_scan(self, msg: LaserScan) -> None:
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
            return

        cx, cy = best
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.pose.position.x = cx
        out.pose.position.y = cy
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
