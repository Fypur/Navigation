import rclpy
from math import copysign
from robot.steady_node import SteadyNode
from msgs.msg import Health, Command, WheelSpeeds, RPMs

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
        self.target_rpm = 0

        self.pub_health = self.create_publisher(Health, "/robot/health", 1)
        self.pub_wheels = self.create_publisher(WheelSpeeds, "/robot/wheels", 10)
        self.sub_encoders = self.create_subscription(RPMs, "/robot/encoders", self.encoders_callback, 10)

        self.create_subscription(Command, "/robot/command", self.cmd_callback, 10)

        # self.create_timer(0.1, self.heartbeat)

        self.t = self.create_timer(0.1, self.update_wheels)

        self.get_logger().info("Control node launched")

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="control"))

    def cmd_callback(self, cmd_msg: Command):
        self.get_logger().info(f"received {cmd_msg.action} with arg {cmd_msg.arg1}")

        wheel_msg = WheelSpeeds()

        if cmd_msg.action == "speed":
            wheel_msg.front_left_wheel_speed = self.match_front_right(self.front_left_RPM, cmd_msg.arg1)
            wheel_msg.front_right_wheel_speed = cmd_msg.arg2
            wheel_msg.back_right_wheel_speed = self.match_front_right(self.back_right_RPM, cmd_msg.arg3)
            wheel_msg.back_left_wheel_speed = self.match_front_right(self.back_left_RPM, cmd_msg.arg4)
            # self.get_logger().info(f"front left wheel adjusted speed : {wheel_msg.front_left_wheel_speed}")
        elif cmd_msg.action == "turn":
            pass  #TODO: depending on angle, send certain values to wheels

        self.lastWheelMsg = wheel_msg

    def encoders_callback(self, msg: RPMs):
        pass

    def front_left_RPM(self, speed: int):
        if speed == 0:
            return 0.0
        return (2.543e-05 * speed**3) - (0.01625 * speed**2) + (3.696 * speed) - 136.4

    def front_right_RPM(self, speed: int):
        if speed == 0:
            return 0.0
        return (2.950e-05 * speed**3) - (0.01849 * speed**2) + (4.246 * speed) - 202.6

    def back_left_RPM(self, speed: int):
        if speed == 0:
            return 0.0
        return (2.307e-05 * speed**3) - (0.01485 * speed**2) + (3.452 * speed) - 123.2

    def back_right_RPM(self, speed: int):
        if speed == 0:
            return 0.0
        return (2.048e-07 * speed**4) - (0.0001161 * speed**3) + (0.01932 * speed**2) - (0.03627 * speed) - 16.98

    def match_front_right(self, rpm_function, speed:int):
        if(speed == 0):
            return 0
        return int(copysign(self.find_matching_wheel_speed(rpm_function, self.front_right_RPM(abs(speed))), speed))

    # https://github.com/Fypur/Navigation/wiki/Encoders#software-based-drive-correction-via-polynomial-regression
    def find_matching_wheel_speed(self, rpm_function, target_RPM : float):
        """
        Finds the required speed for a specific wheel (given its rpm_function) 
        to match a target_RPM.
        """
        if(target_RPM == 0):
            return 0
        # Binary Search to find the required speed
        low = 0.0
        high = 500.0
        tolerance = 1e-5

        while (high - low) > tolerance:
            mid = (low + high) / 2.0
            mid_rpm = rpm_function(mid)

            if mid_rpm < target_RPM:
                low = mid
            else:
                high = mid

        return round((low + high) / 2.0)

    def update_wheels(self):
        #TODO: FAIRE ASSERVISSEMENT ICI AVEC LES ENCODEURS INCREMENTAUX
        #self.get_logger().info("sent speeds " + str(self.lastWheelMsg.wheel1_speed))
        self.pub_wheels.publish(self.lastWheelMsg)


def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
