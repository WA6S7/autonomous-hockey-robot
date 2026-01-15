import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class PuckChaser(Node):
    """
    Chases the detected ball using the PoseStamped produced by lidar_puck_detector.

    Assumption:
      The pose is in the lidar frame, and the lidar frame is aligned with base_link (no yaw offset).
      In your URDF, the lidar joint has no rotation, so this is a valid assumption. 
    """

    def __init__(self):
        super().__init__("puck_chaser")

        # Topics
        self.declare_parameter("target_topic", "/robot1/puck_pose_lidar")
        self.declare_parameter("cmd_vel_topic", "/robot1/cmd_vel")

        # Control gains / limits
        self.declare_parameter("k_lin", 1.4)          # linear speed gain
        self.declare_parameter("k_ang", 3.5)          # angular speed gain
        self.declare_parameter("max_lin", 0.8)        # m/s
        self.declare_parameter("max_ang", 2.5)        # rad/s

        # Behavior tuning
        self.declare_parameter("stop_dist", 0.25)     # stop this far from ball (m)
        self.declare_parameter("align_angle", 0.25)   # deprecated (not used by current pursuit logic)
        self.declare_parameter("stale_timeout", 0.5)  # seconds: if no detection, stop/search
        self.declare_parameter("search_ang", 0.6)     # rad/s when searching

        # More natural pursuit behavior
        self.declare_parameter("allow_reverse", True)  # if True, drive backwards when target is behind
        self.declare_parameter("max_rev", 0.4)          # max reverse speed (m/s) when allow_reverse=True
        self.declare_parameter("min_lin", 0.0)          # minimum forward speed (m/s) once moving (helps overcome stiction)

        target_topic = self.get_parameter("target_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self._last_pose = None
        self._last_pose_time = None

        self.create_subscription(PoseStamped, target_topic, self.on_target, 10)
        self._pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self.on_timer)

        self.get_logger().info(f"Chasing target: {target_topic} -> cmd_vel: {cmd_vel_topic}")

    def on_target(self, msg: PoseStamped) -> None:
        self._last_pose = msg
        self._last_pose_time = self.get_clock().now()

    def on_timer(self) -> None:
        twist = Twist()

        now = self.get_clock().now()
        stale_timeout = float(self.get_parameter("stale_timeout").value)
        search_ang = float(self.get_parameter("search_ang").value)

        # If we have no target or it's stale, rotate slowly to reacquire
        if self._last_pose is None or self._last_pose_time is None:
            twist.angular.z = search_ang
            self._pub.publish(twist)
            return

        age = (now - self._last_pose_time).nanoseconds * 1e-9
        if age > stale_timeout:
            twist.angular.z = search_ang
            self._pub.publish(twist)
            return

        # Target position in lidar frame (aligned with base)
        x = float(self._last_pose.pose.position.x)
        y = float(self._last_pose.pose.position.y)

        dist = math.hypot(x, y)
        bearing = math.atan2(y, x)  # +left, -right

        stop_dist = float(self.get_parameter("stop_dist").value)

        k_lin = float(self.get_parameter("k_lin").value)
        k_ang = float(self.get_parameter("k_ang").value)
        max_lin = float(self.get_parameter("max_lin").value)
        max_ang = float(self.get_parameter("max_ang").value)

        # Always steer toward the ball
        twist.angular.z = clamp(k_ang * bearing, -max_ang, max_ang)

                # Drive while turning (more continuous pursuit):
        # - If the target is in the front half-plane, drive forward while steering.
        # - If the target is behind, either rotate-in-place to face it (default) or reverse toward it.
        allow_reverse = bool(self.get_parameter("allow_reverse").value)
        max_rev = float(self.get_parameter("max_rev").value)
        min_lin = float(self.get_parameter("min_lin").value)

        heading = math.cos(bearing)  # 1.0 straight ahead, 0.0 at +/-90deg, -1.0 straight behind

        if dist > stop_dist:
            if allow_reverse:
                lin_cmd = k_lin * (dist - stop_dist) * heading
                twist.linear.x = clamp(lin_cmd, -max_rev, max_lin)
            else:
                # forward-only: suppress forward motion if the ball is behind the robot
                lin_cmd = k_lin * (dist - stop_dist) * max(0.0, heading)
                twist.linear.x = clamp(lin_cmd, 0.0, max_lin)

            # Optional minimum speed once we intend to move
            if min_lin > 0.0 and abs(twist.linear.x) > 0.0 and abs(twist.linear.x) < min_lin:
                twist.linear.x = min_lin if twist.linear.x > 0.0 else -min_lin
        else:
            twist.linear.x = 0.0

        # If we are close enough, stop
        if dist <= stop_dist:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self._pub.publish(twist)


def main():
    rclpy.init()
    node = PuckChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
