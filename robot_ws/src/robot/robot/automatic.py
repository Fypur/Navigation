import rclpy
from rclpy.node import Node
from msgs.msg import Health, Command, Detect, AutoConfig
import math
from collections import deque

STEP_DISTANCE = 30.0
STEP_ANGLE = 15.0


class Automatic(Node):

    def __init__(self):
        super().__init__("automatic")

        # Publishers
        self.pub_health = self.create_publisher(Health, "/robot/health", 10)
        self.pub_cmd = self.create_publisher(Command, "/robot/control/command", 10)
        self.pub_auto = self.create_publisher(AutoConfig, "/robot/auto/status", 10)

        # Subscribers
        self.create_subscription(AutoConfig, "/robot/auto/config", self.config_callback, 10)
        self.create_subscription(Detect, "/robot/detect/status", self.detect_cb, 10)

        # Timers
        self.create_timer(0.1, self.heartbeat)
        self.create_timer(0.3, self.control_loop)

        # State
        self.stop = True
        self.mode_auto = False
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0

        self.xf = 0.0
        self.yf = 0.0

        self.obstacle = False
        self.avoid_queue = deque()

        self.get_logger().info("Automatic node launched")

    # ------------------------------------------------
    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="auto"))

    # ------------------------------------------------
    def config_callback(self, msg: AutoConfig):
        if msg.action == "config":
            self.mode_auto = msg.mode

            self.x = msg.x
            self.y = msg.y

            self.xf = msg.xf
            self.yf = msg.yf
            self.angle = msg.angle

            self.get_logger().info(
                f"Auto mode={self.mode_auto} angle={self.angle} start=({self.x},{self.y}) goal=({self.xf},{self.yf})")
            self.pub_auto.publish(
                AutoConfig(mode=msg.mode, x=self.x, y=self.y, xf=self.xf, yf=self.yf, angle=self.angle))
            self.stop = False
        elif msg.action == "setPoint":
            self.x = msg.x
            self.y = msg.y
            self.angle = msg.angle

    # ------------------------------------------------
    def detect_cb(self, msg: Detect):
        if (not self.stop and self.mode_auto == True) and msg.action == "objet détecté":
            self.obstacle = True
            self.plan_avoidance()

    # ------------------------------------------------
    def plan_avoidance(self):
        self.avoid_queue.clear()

        sequence = [
            ("turn", 90.0),
            ("forward", 30.0),
            ("turn", -90.0),
            ("forward", 30.0),
            ("turn", -90.0),
            ("forward", 30.0),
            ("turn", 90.0),
        ]

        for action, value in sequence:
            self.avoid_queue.append((action, value))

        self.get_logger().info("Avoidance procedure engaged")

    # ------------------------------------------------
    def control_loop(self):

        if not self.mode_auto or self.stop:
            return

        # Evitement prioritaire
        if self.avoid_queue:
            action, value = self.avoid_queue.popleft()
            self.execute(action, value)
            return

        # Vérifie arrivée
        if self.distance_to_goal() < STEP_DISTANCE:
            self.get_logger().info("Destination reached")
            # self.mode_auto = False
            self.stop = True
            return

        # Navigation simple vers cible
        target_angle = math.degrees(math.atan2(self.yf - self.y, self.xf - self.x))
        angle_diff = self.normalize_angle(target_angle - self.angle)

        if abs(angle_diff) > STEP_ANGLE:
            turn_angle = STEP_ANGLE if angle_diff > 0 else -STEP_ANGLE
            self.execute("turn", turn_angle)
        else:
            self.execute("forward", STEP_DISTANCE)

    # ------------------------------------------------
    def execute(self, action, value):

        msg = Command()

        if action == "forward":
            msg.action = "forward"
            msg.distance = value

            # self.x += math.cos(math.radians(self.angle)) * value
            # self.y += math.sin(math.radians(self.angle)) * value

        elif action == "turn":
            msg.action = "turn"
            msg.angle = value

            # self.angle += msg.angle

        self.pub_cmd.publish(msg)

    # ------------------------------------------------
    def distance_to_goal(self):
        return math.sqrt((self.xf - self.x)**2 + (self.yf - self.y)**2)

    # ------------------------------------------------
    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle


def main():
    rclpy.init()
    node = Automatic()
    rclpy.spin(node)
    rclpy.shutdown()
