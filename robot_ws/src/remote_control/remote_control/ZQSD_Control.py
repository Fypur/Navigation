import rclpy
from rclpy.node import Node
from msgs.msg import RPMs
from rclpy.clock import Clock, ClockType
import pygame

DEFAULT_RPM = 167.0

class ZQSD_Control(Node):

    def __init__(self):
        super().__init__("ZQSD_control")
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)  #IMPORTANT FOR THIS TO WORK ON WSL2

        self.pub_cmd = self.create_publisher(RPMs, "/robot/command", 10)
        self.create_timer(0.1, self.check_input)

        pygame.init()
        self.window = pygame.display.set_mode((300, 300))
        self.clock = pygame.time.Clock()

        self.get_logger().info("Remote control ZQSD node launched")

    def check_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        m = RPMs()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_z]:
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


def main():
    rclpy.init()
    try:
        rclpy.spin(ZQSD_Control())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
