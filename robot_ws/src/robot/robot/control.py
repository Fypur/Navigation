import rclpy
from rclpy.node import Node
from msgs.msg import Health, Command, WheelSpeeds

DEST_X = 700.0
DEST_Y = 550.0


class Control(Node):

    def __init__(self):
        super().__init__("control")

        self.lastWheelMsg = WheelSpeeds(
            wheel1_speed=0,
            wheel2_speed=0,
            wheel3_speed=0,
            wheel4_speed=0,
        )

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)
        self.pub_wheels = self.create_publisher(WheelSpeeds, "/robot/wheels", 1)

        self.create_subscription(Command, "/robot/command", self.cmd_callback, 10)

        self.create_timer(0.1, self.heartbeat)

        self.create_timer(0.1, self.update_wheels)

        self.get_logger().info("Control node launched")

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="control"))

    def cmd_callback(self, cmd_msg: Command):
        #self.get_logger().info("received " + cmd_msg.action)

        wheel_msg = WheelSpeeds()

        if cmd_msg.action == "speed":
            wheel_msg.wheel1_speed = cmd_msg.arg1
            wheel_msg.wheel2_speed = cmd_msg.arg1
            wheel_msg.wheel3_speed = cmd_msg.arg1
            wheel_msg.wheel4_speed = cmd_msg.arg1

        elif cmd_msg.action == "turn":
            pass  #TODO: depending on angle, send certain values to wheels

        self.lastWheelMsg = wheel_msg

    def update_wheels(self):
        #TODO: FAIRE ASSERVISSEMENT ICI AVEC LES ENCODEURS INCREMENTAUX
        #TODO: Send msg to tell the wheels to stop turning after a certain time of not receiving commands
        self.get_logger().info("sent msg " + str(self.lastWheelMsg.wheel1_speed))
        self.pub_wheels.publish(self.lastWheelMsg)


def main():
    rclpy.init()
    rclpy.spin(Control())
    rclpy.shutdown()
