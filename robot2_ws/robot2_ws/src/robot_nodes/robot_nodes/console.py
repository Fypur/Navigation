import rclpy
import threading
import queue
from threading import Lock
from rclpy.node import Node
from robot_msgs.msg import Health, Command, Detect


class Console(Node):

    def __init__(self):
        super().__init__("console")
        self.last_cmd = "forward"
        self.mutex = Lock()
        self.auto_mode = False

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)
        self.pub_cmd = self.create_publisher(Command, "/robot/control/command",
                                             10)

        self.create_subscription(Command, "/robot/control/status",
                                 self.cmdStatus_cb, 10)
        self.create_subscription(Detect, "/robot/detect/status",
                                 self.detect_cb, 10)
        self.create_subscription(Command, "/robot/health/status",
                                 self.health_cb, 10)

        self.create_timer(0.1, self.heartbeat)
        self.create_timer(0.05, self.process_queue)

        # file de commandes thread-safe
        self.cmd_queue = queue.Queue()

        # thread clavier
        threading.Thread(target=self.input_loop, daemon=True).start()

    # -------------------------
    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="console"))

    # -------------------------
    def cmdStatus_cb(self, msg: Command):
        if msg.action == "error":
            self.get_logger().info(f"ERROR: {msg.reason}")
            return

        if msg.action == "mode":
            self.auto_mode = msg.mode
            self.mutex.release()
            return

        if self.auto_mode == False:
            self.get_logger().info(f"EXECUTED: {msg.action}")
            self.mutex.release()

    # -------------------------
    def detect_cb(self, msg: Detect):
        self.get_logger().info(
            f"DETECTION: {msg.action} distance={msg.distance}\r")

    # -------------------------
    def health_cb(self, msg: Command):
        self.get_logger().info(f"HEALTH: {msg.action} reason={msg.reason}\r")

    # ---------------- UI ----------------
    def show_menu(self):
        mode = "auto" if self.auto_mode else "manu"
        print("\n==========================")
        if not self.auto_mode:
            print("Commands: forward | left | right | auto")
        else:
            print("Commands: manu")

        print(f"Mode : {mode}")
        print(f"Last : {self.last_cmd}")
        print("==========================")

    # -------------------------
    def input_loop(self):
        while rclpy.ok():
            self.mutex.acquire()
            self.show_menu()
            cmd = input("> ")
            if cmd == "":
                cmd = self.last_cmd
            else:
                self.last_cmd = cmd
            self.cmd_queue.put(cmd)

    # -------------------------
    def process_queue(self):
        while not self.cmd_queue.empty():

            cmd = self.cmd_queue.get()
            m = Command()

            if self.auto_mode == True and cmd != "manu":
                self.get_logger().info("Command NOT ALLOWED")
                self.mutex.release()
                return

            if cmd == "forward":
                m.action = "forward"
                m.distance = 30.0
            elif cmd == "left":
                m.action = "turn"
                m.angle = -15.0
            elif cmd == "right":
                m.action = "turn"
                m.angle = 15.0
            elif cmd == "auto":
                m.action = "mode"
                m.mode = True
            elif cmd == "manu":
                m.action = "mode"
                m.mode = False
            else:
                self.get_logger().info("Unknown command")
                self.mutex.release()
                return

            self.pub_cmd.publish(m)
            # self.get_logger().info(f"Sent: {m.action}")


def main():
    rclpy.init()
    rclpy.spin(Console())
    rclpy.shutdown()
