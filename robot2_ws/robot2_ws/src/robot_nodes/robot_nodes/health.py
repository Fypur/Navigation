import rclpy
import collections
from rclpy.node import Node
from robot_msgs.msg import Health, Command

MODULES = ["Lidar", "detect", "console", "control", "driver", "auto"]


class HealthNode(Node):

    def __init__(self):
        super().__init__("health")

        self.pub = self.create_publisher(Command, "/robot/health/status", 10)
        self.create_subscription(Health, "/robot/health", self.cb, 10)

        self.miss = collections.defaultdict(int)
        self.create_timer(0.1, self.check)

    def cb(self, msg):
        self.miss[msg.name] = 0

    def check(self):
        for m in MODULES:
            self.miss[m] += 1
            if self.miss[m] >= 5:
                self.pub.publish(
                    Command(action="error", reason=f"{m} missing"))


def main():
    rclpy.init()
    rclpy.spin(HealthNode())
    rclpy.shutdown()
