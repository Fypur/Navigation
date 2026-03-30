import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import Health, Command, WheelSpeeds

DEST_X = 700.0
DEST_Y = 550.0


class Control(SteadyNode):

    def __init__(self):
        super().__init__("control")

        self.lastWheelMsg = WheelSpeeds(
            wheel1_speed=0,
            wheel2_speed=0,
            wheel3_speed=0,
            wheel4_speed=0,
        )

        self.pub_health = self.create_publisher(Health, "/robot/health", 1)
        self.pub_wheels = self.create_publisher(WheelSpeeds, "/robot/wheels", 10)

        self.create_subscription(Command, "/robot/command", self.cmd_callback, 10)

        # self.create_timer(0.1, self.heartbeat)

        self.i = 0
        self.t = self.create_timer(0.1, self.update_wheels)

        self.get_logger().info("Control node launched")

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="control"))

    def cmd_callback(self, cmd_msg: Command):
        self.get_logger().info(f"received {cmd_msg.action} with arg {cmd_msg.arg1}")

        wheel_msg = WheelSpeeds()

        if cmd_msg.action == "speed":
            wheel_msg.front_left_wheel_speed = cmd_msg.arg1
            wheel_msg.front_right_wheel_speed = cmd_msg.arg2
            wheel_msg.back_right_wheel_speed = cmd_msg.arg3
            wheel_msg.back_left_wheel_speed = cmd_msg.arg4

        elif cmd_msg.action == "turn":
            pass  #TODO: depending on angle, send certain values to wheels

        self.lastWheelMsg = wheel_msg

    def update_wheels(self):
        #TODO: FAIRE ASSERVISSEMENT ICI AVEC LES ENCODEURS INCREMENTAUX
        #TODO: Send msg to tell the wheels to stop turning after a certain time of not receiving commands
        #self.get_logger().info("sent speeds " + str(self.lastWheelMsg.wheel1_speed))
        self.pub_wheels.publish(self.lastWheelMsg)


def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
