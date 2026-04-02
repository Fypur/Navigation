import rclpy
from rclpy.node import Node
from msgs.msg import Command
from rclpy.clock import Clock, ClockType
import pygame

SPEED = 200

class ZQSD_Control(Node):

    def __init__(self):
        super().__init__("ZQSD_control")
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)  #IMPORTANT FOR THIS TO WORK ON WSL2

        self.pub_cmd = self.create_publisher(Command, "/robot/command", 10)
        self.create_timer(0.1, self.check_input)

        pygame.init()
        self.window = pygame.display.set_mode((300, 300))
        self.clock = pygame.time.Clock()

        self.get_logger().info("Remote control ZQSD node launched")

    def check_input(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        m = Command()
        m.action = "speed"

        keys = pygame.key.get_pressed()
        if keys[pygame.K_z]:
            m.arg1 = SPEED
            m.arg2 = SPEED
            m.arg3 = SPEED
            m.arg4 = SPEED
        elif keys[pygame.K_s]:
            m.arg1 = -SPEED
            m.arg2 = -SPEED
            m.arg3 = -SPEED
            m.arg4 = -SPEED
        elif keys[pygame.K_q]:
            m.arg1 = -SPEED
            m.arg2 = SPEED
            m.arg3 = -SPEED
            m.arg4 = SPEED
        elif keys[pygame.K_d]:
            m.arg1 = SPEED
            m.arg2 = -SPEED
            m.arg3 = SPEED
            m.arg4 = -SPEED
        elif keys[pygame.K_a]:
            m.arg1 = -SPEED
            m.arg2 = SPEED
            m.arg3 = SPEED
            m.arg4 = -SPEED
        elif keys[pygame.K_e]:
            m.arg1 = SPEED
            m.arg2 = -SPEED
            m.arg3 = -SPEED
            m.arg4 = SPEED
        else:
            m.arg1 = 0
            m.arg2 = 0
            m.arg3 = 0
            m.arg4 = 0

        self.pub_cmd.publish(m)
        self.get_logger().info(f"Sent {m.action} with arg {m.arg1}, {m.arg2}, {m.arg3}, {m.arg4}")


def main():
    rclpy.init()
    try:
        rclpy.spin(ZQSD_Control())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
