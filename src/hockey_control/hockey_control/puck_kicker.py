import rclpy
from rclpy.node import Node
import subprocess
import time

class PuckKicker(Node):
    def __init__(self):
        super().__init__('puck_kicker')
        self.get_logger().info("PuckKicker node started. Waiting 5s before kicking...")
        self.timer = self.create_timer(5.0, self.start_kicking)
        self.kick_count = 0
        self.max_kicks = 20 # Kick for ~2 seconds (if timer is 0.1s)

    def start_kicking(self):
        self.destroy_timer(self.timer)
        self.get_logger().info("Starting kick burst!")
        # Create a faster timer for the burst
        self.kick_timer = self.create_timer(0.1, self.kick_burst)

    def kick_burst(self):
        if self.kick_count >= self.max_kicks:
            self.get_logger().info("Kick burst finished.")
            self.destroy_timer(self.kick_timer)
            return

        self.apply_wrench('puck', 'MODEL')
        self.apply_wrench('puck::puck_link', 'LINK') # Try both to be sure
        
        self.kick_count += 1

    def apply_wrench(self, entity_name, entity_type):
        # Increased force to 500N
        cmd = [
            "gz", "service",
            "-s", "/world/hockey_world/wrench",
            "--reqtype", "gz.msgs.EntityWrench",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "500",
            f"--req", f'entity: {{name: "{entity_name}", type: {entity_type}}}, wrench: {{force: {{x: 500.0, y: 500.0, z: 0.0}}}}'
        ]
        
        try:
            # We don't wait long, firing and forgetting mostly
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f"Failed to execute gz service: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PuckKicker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
