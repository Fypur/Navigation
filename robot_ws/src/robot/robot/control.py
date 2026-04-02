import rclpy
from math import copysign
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
        self.target_rpm = 0

        self.pub_health = self.create_publisher(Health, "/robot/health", 1)
        self.pub_wheels = self.create_publisher(WheelSpeeds, "/robot/wheels", 10)

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
            if(abs(cmd_msg.arg1) == abs(cmd_msg.arg2) == abs(cmd_msg.arg3) == abs(cmd_msg.arg4)):
                [fl_speed, fr_speed, br_speed, bl_speed], target_rpm = self.find_matching_speeds(abs(cmd_msg.arg1))
                wheel_msg.front_left_wheel_speed = int(copysign(fl_speed, cmd_msg.arg1))
                wheel_msg.front_right_wheel_speed = int(copysign(fr_speed, cmd_msg.arg2))
                wheel_msg.back_right_wheel_speed = int(copysign(br_speed, cmd_msg.arg3))
                wheel_msg.back_left_wheel_speed = int(copysign(bl_speed, cmd_msg.arg4))
                self.target_rpm = target_rpm
            else:
                wheel_msg.front_left_wheel_speed = cmd_msg.arg1
                wheel_msg.front_right_wheel_speed = cmd_msg.arg2
                wheel_msg.back_right_wheel_speed = cmd_msg.arg3
                wheel_msg.back_left_wheel_speed = cmd_msg.arg4

        elif cmd_msg.action == "turn":
            pass  #TODO: depending on angle, send certain values to wheels

        self.lastWheelMsg = wheel_msg

    def front_left_RPM(self, speed):
        return (2.543e-05 * speed**3) - (0.01625 * speed**2) + (3.696 * speed) - 136.4

    def front_right_RPM(self, speed):
        return (2.950e-05 * speed**3) - (0.01849 * speed**2) + (4.246 * speed) - 202.6

    def back_left_RPM(self, speed):
        return (2.307e-05 * speed**3) - (0.01485 * speed**2) + (3.452 * speed) - 123.2

    def back_right_RPM(self, speed):
        return (2.048e-07 * speed**4) - (0.0001161 * speed**3) + (0.01932 * speed**2) - (0.03627 * speed) - 16.98

    def find_matching_speeds(self, target_speed: int):
        """
        Given a speed, calculates the target RPM using the Front Right wheel,
        then uses binary search to find the required speeds for the other 
        wheels to achieve that exact same RPM.
        """
        # 1. Calculate the target RPM we want all wheels to match
        target_rpm = self.front_right_RPM(target_speed)

        # Helper function: Binary Search for continuous functions
        def binary_search_speed(rpm_function, target: float, low=0.0, high=500.0, tolerance=1e-5):
            """
            Finds the speed that results in the target RPM.
            Assumes the RPM function is monotonically increasing between 'low' and 'high'.
            """
            while (high - low) > tolerance:
                mid = (low + high) / 2.0
                mid_rpm = rpm_function(mid)

                if mid_rpm < target:
                    low = mid  # Target is in the upper half
                else:
                    high = mid # Target is in the lower half

            return (low + high) / 2.0

        # 2. Use binary search to find the required speeds for the other wheels
        fl_speed: float = binary_search_speed(self.front_left_RPM, target_rpm)
        bl_speed: float = binary_search_speed(self.back_left_RPM, target_rpm)
        br_speed: float = binary_search_speed(self.back_right_RPM, target_rpm)

        return [round(fl_speed), target_speed, round(br_speed), round(bl_speed)], target_rpm

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
