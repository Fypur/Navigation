import rclpy
from rclpy.node import Node
from msgs.msg import RPMs, WheelSpeeds
from rclpy.clock import Clock, ClockType
import pygame
from enum import IntEnum

DEFAULT_RPM = 167.0
DEFAULT_PWM = 240

class ZQSD_Control(Node):

    class WheelDir(IntEnum):
        BACK = -1
        STOP = 0
        FORWARD = 1

    def __init__(self):
        super().__init__("ZQSD_control")
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)  #IMPORTANT FOR THIS TO WORK ON WSL2

        self.pub_cmd = self.create_publisher(RPMs, "/robot/command", 10)
        self.pub_pwm = self.create_publisher(WheelSpeeds, "/robot/raw_command", 10)
        self.create_timer(0.1, self.check_input_pwm)

        pygame.init()
        self.window = pygame.display.set_mode((300, 300))
        self.clock = pygame.time.Clock()

        self.get_logger().info("Remote control ZQSD node launched")

    def send_pwm(self, front_left: int, front_right: int, back_right: int, back_left: int)
        msg = WheelSpeeds()
        msg.front_left_wheel_speed = front_left
        msg.front_right_wheel_speed = front_right
        msg.back_right_wheel_speed = back_right
        msg.back_left_wheel_speed = back_left
        self.pub_pwm.publish(msg)

    def send_rpm(self, front_left: float, front_right: float, back_right: float, back_left: float)
        msg = RPMs()
        msg.front_left_rpm = front_left
        msg.front_right_rpm = front_right
        msg.back_right_rpm = back_right
        msg.back_left_rpm = back_left
        self.pub_cmd.publish(msg)

    def set_wheel_dirs(self, is_rpm: bool, front_left: WheelDir, front_right: WheelDir, back_right: WheelDir, back_left: WheelDir):
        if is_rpm:
            self.send_rpm(front_left, front_right, back_right, back_left)
        else:
            self.send_pwm(front_left, front_right, back_right, back_left)


    def check_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        m = RPMs()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_z]:
            self.set_wheel_dirs(True, )
            m.front_left_rpm = DEFAULT_RPM
            m.front_right_rpm = DEFAULT_RPM
            m.back_right_rpm = DEFAULT_RPM
            m.back_left_rpm = DEFAULT_RPM
        elif keys[pygame.K_s]:
            m.front_left_rpm = -DEFAULT_RPM
            m.front_right_rpm = -DEFAULT_RPM
            m.back_right_rpm = -DEFAULT_RPM
            m.back_left_rpm = -DEFAULT_RPM
        elif keys[pygame.K_q]:
            m.front_left_rpm = -DEFAULT_RPM
            m.front_right_rpm = DEFAULT_RPM
            m.back_right_rpm = -DEFAULT_RPM
            m.back_left_rpm = DEFAULT_RPM
        elif keys[pygame.K_d]:
            m.front_left_rpm = DEFAULT_RPM
            m.front_right_rpm = -DEFAULT_RPM
            m.back_right_rpm = DEFAULT_RPM
            m.back_left_rpm = -DEFAULT_RPM
        elif keys[pygame.K_a]:
            m.front_left_rpm = -DEFAULT_RPM
            m.front_right_rpm = DEFAULT_RPM
            m.back_right_rpm = DEFAULT_RPM
            m.back_left_rpm = -DEFAULT_RPM
        elif keys[pygame.K_e]:
            m.front_left_rpm = DEFAULT_RPM
            m.front_right_rpm = -DEFAULT_RPM
            m.back_right_rpm = -DEFAULT_RPM
            m.back_left_rpm = DEFAULT_RPM
        else:
            m.front_left_rpm = 0.0
            m.front_right_rpm = 0.0
            m.back_right_rpm = 0.0
            m.back_left_rpm = 0.0

        self.pub_cmd.publish(m)
        self.get_logger().info(f"Sent RPMs {m.front_left_rpm}, {m.front_right_rpm}, {m.back_right_rpm}, {m.back_left_rpm}")


    def check_input_pwm(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        m = WheelSpeeds()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_z]:
            m.front_left_wheel_speed = DEFAULT_PWM
            m.front_right_wheel_speed = DEFAULT_PWM
            m.back_right_wheel_speed = DEFAULT_PWM
            m.back_left_wheel_speed = DEFAULT_PWM
        elif keys[pygame.K_s]:
            m.front_left_wheel_speed = -DEFAULT_PWM
            m.front_right_wheel_speed = -DEFAULT_PWM
            m.back_right_wheel_speed = -DEFAULT_PWM
            m.back_left_wheel_speed = -DEFAULT_PWM
        elif keys[pygame.K_q]:
            m.front_left_wheel_speed = -DEFAULT_PWM
            m.front_right_wheel_speed = DEFAULT_PWM
            m.back_right_wheel_speed = -DEFAULT_PWM
            m.back_left_wheel_speed = DEFAULT_PWM
        elif keys[pygame.K_d]:
            m.front_left_wheel_speed = DEFAULT_PWM
            m.front_right_wheel_speed = -DEFAULT_PWM
            m.back_right_wheel_speed = DEFAULT_PWM
            m.back_left_wheel_speed = -DEFAULT_PWM
        elif keys[pygame.K_a]:
            m.front_left_wheel_speed = -DEFAULT_PWM
            m.front_right_wheel_speed = DEFAULT_PWM
            m.back_right_wheel_speed = DEFAULT_PWM
            m.back_left_wheel_speed = -DEFAULT_PWM
        elif keys[pygame.K_e]:
            m.front_left_wheel_speed = DEFAULT_PWM
            m.front_right_wheel_speed = -DEFAULT_PWM
            m.back_right_wheel_speed = -DEFAULT_PWM
            m.back_left_wheel_speed = DEFAULT_PWM
        else:
            m.front_left_wheel_speed = 0
            m.front_right_wheel_speed = 0
            m.back_right_wheel_speed = 0
            m.back_left_wheel_speed = 0

        self.pub_pwm.publish(m)
        self.get_logger().info(
            f"Sent PWMs {m.front_left_wheel_speed}, {m.front_right_wheel_speed}, {m.back_right_wheel_speed}, {m.back_left_wheel_speed}")


def main():
    rclpy.init()
    try:
        rclpy.spin(ZQSD_Control())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
