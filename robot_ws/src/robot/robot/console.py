import rclpy
import threading
import queue
from robot.steady_node import SteadyNode
from msgs.msg import Health, Command

SPEED = 100
TURN_ANGLE = 15.0


class Console(SteadyNode):

    def __init__(self):
        super().__init__("console")
        self.last_cmd = "forward"

        self.pub_health = self.create_publisher(Health, "/robot/health", 1)

        self.pub_cmd = self.create_publisher(Command, "/robot/command", 10)

        self.create_timer(0.1, self.heartbeat)

        self.input_loop()

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="console"))

    # ---------------- UI ----------------
    def show_menu(self):
        log = "\n==========================\n"
        log += "Commands: forward | backward | left | right | stop\n"
        log += f"Last : {self.last_cmd}\n"
        log += "==========================\n"
        self.get_logger().info(log)
        print(log)

    def input_loop(self):
        while rclpy.ok():
            self.show_menu()
            cmd = input("> ")

            if cmd == "":
                cmd = self.last_cmd
            else:
                self.last_cmd = cmd

            self.process(cmd)

    def process(self, cmd: str):
        m = Command()

        if cmd == "forward":
            m.action = "speed"
            m.arg1 = SPEED  #this represents speed, in a byte format
            #TODO:  Change cmd line to take an argument in speed
        elif cmd == "backward":
            m.action = "speed"
            m.arg1 = -SPEED
        elif cmd == "stop":
            m.action = "speed"
            m.arg1 = 0
        elif cmd == "left":
            m.action = "turn"
            m.arg1 = -TURN_ANGLE
        elif cmd == "right":
            m.action = "turn"
            m.arg1 = TURN_ANGLE
        else:
            self.get_logger().info("Unknown command")
            return

        self.pub_cmd.publish(m)
        #self.get_logger().info(f"Sent: {m.action} with arg1 {m.arg1}")


def main():
    rclpy.init()
    rclpy.spin(Console())
    rclpy.shutdown()
