import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import Health, Command

SPEED = 200
TURN_ANGLE = 15.0


class Console(SteadyNode):

    def __init__(self):
        super().__init__("console")
        self.last_cmd = "forward"

        self.pub_health = self.create_publisher(Health, "/robot/health", 1)

        self.pub_cmd = self.create_publisher(Command, "/robot/command", 10)

        self.create_timer(0.1, self.heartbeat)

        # This is technically bad since it blocks the main thread
        # But I had issues with the previous version with some desyncs
        # Console shouldn't really be receiving data anyways so i'd much
        # rather leave it like this. Feel free to change it though
        self.input_loop()

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="console"))

    # ---------------- UI ----------------
    def show_menu(self):
        log = "\n==========================\n"
        log += "Commands: speed | stop\n"
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

        split_cmd = cmd.split(" ")
        command_name = split_cmd[0]

        if command_name == "speed":
            m.action = "speed"
            if len(split_cmd) >= 2:
                m.arg1 = int(split_cmd[1])
            else:
                m.arg1 = SPEED
        elif command_name == "stop":
            m.action = "speed"
            m.arg1 = 0
        else:
            self.get_logger().info("Unknown command")
            return

        self.pub_cmd.publish(m)
        #self.get_logger().info(f"Sent: {m.action} with arg1 {m.arg1}")


def main():
    rclpy.init()
    rclpy.spin(Console())
    rclpy.shutdown()

main()