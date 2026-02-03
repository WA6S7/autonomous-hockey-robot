import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import math
import time

class SimpleFSM(Node):
    def __init__(self):
        super().__init__('simple_fsm')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('puck_topic', 'puck_pose_lidar')
        self.declare_parameter('goal_y', 0.0)
        
        self.robot_name = self.get_parameter('robot_name').value
        puck_topic_name = self.get_parameter('puck_topic').value
        self.goal_y = self.get_parameter('goal_y').value
        

        self.puck_sub = self.create_subscription(
            PoseStamped,
            puck_topic_name,
            self.puck_callback,
            10
        )
        
        # Scan for obstacle avoidance
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # State
        self.last_puck_time = 0.0
        self.puck_x = 0.0
        self.puck_y = 0.0
        self.puck_visible = False
        
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        
        # FSM
        self.state = "SEARCH" # SEARCH, APPROACH, AVOID
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"SimpleFSM started for {self.robot_name}")

    def puck_callback(self, msg: PoseStamped):
        self.last_puck_time = time.time()
        self.puck_x = msg.pose.position.x
        self.puck_y = msg.pose.position.y
        self.puck_visible = True

    def scan_callback(self, msg: LaserScan):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def control_loop(self):
        if time.time() - self.last_puck_time > 1.0:
            self.puck_visible = False

        obstacle_detected = False
        if self.ranges:
            puck_angle = math.atan2(self.puck_y, self.puck_x)
            puck_dist = math.hypot(self.puck_x, self.puck_y)
            
            for i, r in enumerate(self.ranges):
                if math.isinf(r) or math.isnan(r) or r > 1.0:
                    continue
                
                # Calculate angle of this lidar point
                angle = self.angle_min + i * self.angle_increment
                
                # Normalize angle to -pi to pi
                angle = math.atan2(math.sin(angle), math.cos(angle))
                
                if self.puck_visible and abs(angle - puck_angle) < 0.35:
                    if abs(r - puck_dist) < 0.3:
                        continue
                
                # If something else is very close (e.g. another robot or wall)
                if r < 0.35:
                    obstacle_detected = True
                    break

        # Determine State
        if obstacle_detected:
            self.state = "AVOID"
        elif self.puck_visible:
            self.state = "APPROACH"
        else:
            self.state = "SEARCH"

        twist = Twist()

        if self.state == "SEARCH":
            # Rotate
            twist.angular.z = 0.8
        
        elif self.state == "APPROACH":
            # Simple P-controller
            # Turn towards y
            kp_ang = 2.0
            twist.angular.z = kp_ang * math.atan2(self.puck_y, self.puck_x)
            
            # Drive forward
            # Slow down if turning implies we are not facing it?
            # Or just drive.
            kp_lin = 1.0
            # If we are far, go fast. If close, ram it.
            twist.linear.x = min(0.8, kp_lin * self.puck_x)
            # Ensure we always have some forward speed to hit it
            if twist.linear.x < 0.2:
                 twist.linear.x = 0.3

        elif self.state == "AVOID":
            # Back up and turn
            twist.linear.x = -0.15
            twist.angular.z = 1.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
