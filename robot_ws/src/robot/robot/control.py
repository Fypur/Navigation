import rclpy
from math import copysign
from robot.steady_node import SteadyNode
from msgs.msg import Health, Command, WheelSpeeds, RPMs
from time import time
from typing import Callable

DEST_X = 700.0
DEST_Y = 550.0


class Control(SteadyNode):

    class WheelControl:
        # PID pour une roue
        # Ce PID n'est là que pour compenser des petites erreurs, la formule finale pour calc la PMW est
        # PWM = int(basePWM + WheelControl.calcPWM())
        def __init__(self, basePWMFunction: Callable[[float], float]) -> None:
            self.kp = 0.1
            self.ki = 0.01
            self.kd = 0.05
            self.accumulated_error = 0.0
            self.last_error = 0.0
            self.max_accumulated_error = 10.0
            self.min_accumulated_error = -10.0
            self.last_time = time()
            self.basePWMFunction = basePWMFunction


        def calcPWM(self, target_rpm: float, current_rpm: float) -> int:
            t = time()
            deltaTime = t - self.last_time
            self.last_time = t

            error = target_rpm - current_rpm
            
            self.accumulated_error += error * deltaTime
            if self.accumulated_error > self.max_accumulated_error:
                self.accumulated_error = self.max_accumulated_error
            elif self.accumulated_error < self.min_accumulated_error:
                self.accumulated_error = self.min_accumulated_error

            derivative = (error - self.last_error) / deltaTime

            pwm = int(self.basePWMFunction(current_rpm) + self.kp * error + self.ki * self.accumulated_error + self.kd * derivative)
            
            if pwm > 255:
                pwm = 255
            elif pwm < -255:
                pwm = -255

            return pwm

            

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

        self.create_subscription(RPMs, "/robot/command", self.cmd_callback, 10)

        # self.create_timer(0.1, self.heartbeat)

        self.t = self.create_timer(0.1, self.update_wheels)

        # init asservissement
        """self.wheelControls = [
            self.WheelControl(lambda rpm: self.match_front_right(self.front_left_RPM, rpm))
        ]"""

        self.last_encoders_msg = RPMs()

        self.get_logger().info("Control node launched")

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="control"))

    def cmd_callback(self, cmd_msg: RPMs):
        # self.get_logger().info(f"received {cmd_msg.action} with arg {cmd_msg.arg1}")

        wheel_msg = WheelSpeeds()

        wheel_msg.front_left_wheel_speed = self.binary_search(self.front_left_RPM, cmd_msg.front_left_rpm)
        wheel_msg.front_right_wheel_speed = self.binary_search(self.front_right_RPM, cmd_msg.front_right_rpm)
        wheel_msg.back_right_wheel_speed = self.binary_search(self.back_right_RPM, cmd_msg.back_right_rpm)
        wheel_msg.back_left_wheel_speed = self.binary_search(self.back_left_RPM, cmd_msg.back_left_rpm)


        """if cmd_msg.action == "speed":
            wheel_msg.front_left_wheel_speed = self.match_front_right(self.front_left_RPM, cmd_msg.arg1)
            wheel_msg.front_right_wheel_speed = cmd_msg.arg2
            wheel_msg.back_right_wheel_speed = self.match_front_right(self.back_right_RPM, cmd_msg.arg3)
            wheel_msg.back_left_wheel_speed = self.match_front_right(self.back_left_RPM, cmd_msg.arg4)
            # self.get_logger().info(f"front left wheel adjusted speed : {wheel_msg.front_left_wheel_speed}")
        elif cmd_msg.action == "turn":
            pass  #TODO: depending on angle, send certain values to wheels"""

        self.lastWheelMsg = wheel_msg

    def encoders_callback(self, msg: RPMs):
        self.last_encoders_msg = msg

    def update_wheels(self):
        #TODO: FAIRE ASSERVISSEMENT ICI AVEC LES ENCODEURS INCREMENTAUX
        #self.get_logger().info("sent speeds " + str(self.lastWheelMsg.wheel1_speed))


        self.pub_wheels.publish(self.lastWheelMsg)

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

    def match_front_right(self, rpm_function, speed: int):
        if (speed == 0):
            return 0
        return copysign(self.find_matching_wheel_speed(rpm_function, self.front_right_RPM(abs(speed))), speed)

    # https://github.com/Fypur/Navigation/wiki/Encoders#software-based-drive-correction-via-polynomial-regression
    def find_matching_wheel_speed(self, rpm_function, target_RPM: float):
        """
        Finds the required speed for a specific wheel (given its rpm_function) 
        to match a target_RPM.
        """
        if (target_RPM == 0):
            return 0.
        
        return self.binary_search(rpm_function, target_RPM)   

    def binary_search(self, func, target):
        # Binary Search to find the required speed
        low = 0.0
        high = 500.0
        tolerance = 1e-2

        while (high - low) > tolerance:
            mid = (low + high) / 2.0
            mid_rpm = func(mid)

            if mid_rpm < target:
                low = mid
            else:
                high = mid

        return (low + high) / 2.0

def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
