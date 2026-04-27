import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import RPMs

DEFAULT_RPM = 150.0


class Console(SteadyNode):

    def __init__(self):
        super().__init__("console")
        self.last_cmd = "setrpm"

        self.pub_cmd = self.create_publisher(RPMs, "/robot/command", 10)

        # This is technically bad since it blocks the main thread
        # But I had issues with the previous version with some desyncs
        # Console shouldn't really be receiving data anyways so i'd much
        # rather leave it like this. Feel free to change it though
        self.input_loop()

    # ---------------- UI ----------------
    def show_menu(self):
        log = "\n==========================\n"
        log += "Commands: setrpm | stop\n"
        log += f"Last : {self.last_cmd}\n"
        log += "==========================\n"
        self.get_logger().info(log)

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
        m = RPMs()

        split_cmd = cmd.strip().split(" ")
        command_name = split_cmd[0]

        def get_arg(arg_index: int):
            return float(split_cmd[arg_index])

        if command_name == "setrpm":
            if len(split_cmd) == 1:
                m.front_left_rpm = DEFAULT_RPM
                m.front_right_rpm = DEFAULT_RPM
                m.back_right_rpm = DEFAULT_RPM
                m.back_left_rpm = DEFAULT_RPM
            elif len(split_cmd) == 2:
                m.front_left_rpm = get_arg(1)
                m.front_right_rpm = get_arg(1)
                m.back_right_rpm = get_arg(1)
                m.back_left_rpm = get_arg(1)
            elif len(split_cmd) == 5:
                m.front_left_rpm = get_arg(1)
                m.front_right_rpm = get_arg(2)
                m.back_right_rpm = get_arg(3)
                m.back_left_rpm = get_arg(4)
            else:
                self.get_logger().error(f"The setrpm command either takes none, one or four arguments")
        elif command_name == "stop":
            m.front_left_rpm = 0.
            m.front_right_rpm = 0.
            m.back_right_rpm = 0.
            m.back_left_rpm = 0.
        else:
            self.get_logger().error(f"Unknown command \"{command_name}\"")
            return

        self.pub_cmd.publish(m)
        #self.get_logger().info(f"Sent: {m.action} with arg1 {m.arg1}")


def main():
    rclpy.init()
    rclpy.spin(Console())
    rclpy.shutdown()

main()
