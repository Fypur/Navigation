import rclpy
import threading
import queue
from rclpy.node import Node
from msgs.msg import Health, Command


class Console(Node):

    def __init__(self):
        super().__init__("console")
        self.last_cmd = "forward"

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)

        self.pub_cmd = self.create_publisher(Command, "/robot/command", 30)

        self.create_timer(0.1, self.heartbeat)
        self.create_timer(0.05, self.process_queue)

        # file de commandes thread-safe
        self.cmd_queue = queue.Queue()

        # thread clavier
        threading.Thread(target=self.input_loop, daemon=True).start()

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
            self.cmd_queue.put(cmd)

    def process_queue(self):
        while not self.cmd_queue.empty():

            cmd = self.cmd_queue.get()
            m = Command()

            if cmd == "forward":
                m.action = "speed"
                m.arg1 = 10  #this represents speed, in a byte format
                #TODO:  Change cmd line to take an argument in speed
            elif cmd == "backward":
                m.action = "speed"
                m.arg1 = -10
            elif cmd == "stop":
                m.action = "speed"
                m.arg1 = 0
            elif cmd == "left":
                m.action = "turn"
                m.arg1 = -15.0  #turn angle
            elif cmd == "right":
                m.action = "turn"
                m.arg1 = 15.0
            else:
                self.get_logger().info("Unknown command")
                return

            self.pub_cmd.publish(m)
            #self.get_logger().info(f"Sent: {m.action} with arg1 {m.arg1}")


def main():
    rclpy.init()
    rclpy.spin(Console())
    rclpy.shutdown()
