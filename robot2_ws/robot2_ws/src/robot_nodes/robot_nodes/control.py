import rclpy
import math
from threading import Lock
from rclpy.node import Node
from robot_msgs.msg import Health, Command, AutoConfig

DEST_X = 700.0
DEST_Y = 550.0


class Control(Node):

    def __init__(self):
        super().__init__("control")
        self.last_cmd = None
        self.mutex = Lock()
        self.mode = False  #False manu True Auto
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)
        self.pub_driver = self.create_publisher(Command,
                                                "/robot/driver/command", 10)
        self.pub_status = self.create_publisher(Command,
                                                "/robot/control/status", 10)

        self.create_subscription(Command, "/robot/control/command",
                                 self.cmd_cb, 10)
        self.create_subscription(Command, "/robot/driver/status",
                                 self.driver_cb, 10)
        self.create_subscription(Command, "/robot/health/status",
                                 self.health_cb, 10)
        self.pub_auto = self.create_publisher(AutoConfig, "/robot/auto/config",
                                              10)
        self.create_subscription(AutoConfig, "/robot/auto/status",
                                 self.auto_cb, 10)

        self.create_timer(0.1, self.heartbeat)

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="control"))

    def auto_cb(self, msg):
        self.pub_status.publish(Command(action=self.last_cmd, mode=self.mode))
        self.mutex.release()

    def cmd_cb(self, msg):
        if msg.action == "forward":
            self.mutex.acquire()
            self.last_cmd = msg.action
            self.x += math.cos(math.radians(self.angle)) * msg.distance
            self.y += math.sin(math.radians(self.angle)) * msg.distance
            self.pub_driver.publish(
                Command(action="run", distance=msg.distance))
            if self.mode == True:
                self.pub_auto.publish(
                    AutoConfig(action="setPoint",
                               x=self.x,
                               y=self.y,
                               angle=self.angle))
        elif msg.action == "turn":
            self.mutex.acquire()
            self.last_cmd = msg.action
            self.angle += msg.angle
            self.pub_driver.publish(Command(action="turn", angle=msg.angle))
            if self.mode == True:
                self.pub_auto.publish(
                    AutoConfig(action="setPoint",
                               x=self.x,
                               y=self.y,
                               angle=self.angle))
        elif msg.action == "mode":
            self.mutex.acquire()
            self.mode = msg.mode
            self.last_cmd = msg.action
            self.pub_auto.publish(
                AutoConfig(action="config",
                           mode=msg.mode,
                           x=self.x,
                           y=self.y,
                           xf=DEST_X,
                           yf=DEST_Y,
                           angle=self.angle))

    def driver_cb(self, msg):
        self.pub_status.publish(Command(action=self.last_cmd))
        self.mutex.release()

    def health_cb(self, msg):
        if msg.action == "error":
            self.pub_status.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(Control())
    rclpy.shutdown()
