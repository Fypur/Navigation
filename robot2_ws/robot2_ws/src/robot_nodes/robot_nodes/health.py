import rclpy
import collections
from rclpy.node import Node
from robot_msgs.msg import Health, Command

MODULES = ["Lidar", "detect", "console", "control", "driver", "auto"]


class HealthNode(Node):

    def __init__(self):
        super().__init__("health")

        self.pub = self.create_publisher(Command, "/robot/health/status", 10)
        self.create_subscription(Health, "/robot/health", self.reset_health_callback, 10)

        self.miss = collections.defaultdict(int)
        self.create_timer(0.1, self.check_robot_health)

        self.get_logger().info("Health node launched")

    def reset_health_callback(self, msg: Health):
        self.miss[msg.name] = 0

    def check_robot_health(self):
        for m in MODULES:
            self.miss[m] += 1
            if self.miss[m] >= 5:
                self.pub.publish(Command(action="error", reason=f"{m} missing"))


def main():
    rclpy.init()
    rclpy.spin(HealthNode())
    rclpy.shutdown()
