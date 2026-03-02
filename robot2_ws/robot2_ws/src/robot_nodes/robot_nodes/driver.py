import rclpy
import math
import pygame
from rclpy.node import Node
from robot_msgs.msg import Health, Command

WIDTH = 800
HEIGHT = 600

DEST_X = 700
DEST_Y = 550


class Driver(Node):

    def __init__(self):
        super().__init__("driver")

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)
        self.pub_status = self.create_publisher(Command, "/robot/driver/status", 10)

        self.create_subscription(Command, "/robot/driver/command", self.cmd_callback, 10)

        # heartbeat
        self.create_timer(0.1, self.heartbeat)

        # pygame loop 30 FPS
        self.create_timer(1 / 30, self.update_screen)

        # ---- PYGAME INIT ----
        pygame.init()
        pygame.display.set_caption("Robot Driver")
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()

        # robot state
        self.x = 0  # WIDTH/2
        self.y = 0  # HEIGHT-80
        self.angle = 0  # -90

    # ----------------------
    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="driver"))

    # ----------------------
    def cmd_callback(self, msg: Command):
        if msg.action == "run":
            self.x += math.cos(math.radians(self.angle)) * msg.distance
            self.y += math.sin(math.radians(self.angle)) * msg.distance

        elif msg.action == "turn":
            self.angle += msg.angle

        self.pub_status.publish(Command(action=msg.action))

    # ----------------------
    def update_screen(self):

        # important sinon fenêtre freeze
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return

        self.screen.fill((15, 15, 20))

        # draw the target
        pygame.draw.circle(self.screen, (255, 0, 0), (int(DEST_X), int(DEST_Y)), 20)

        # robot body
        pygame.draw.circle(self.screen, (0, 255, 120), (int(self.x), int(self.y)), 12)

        # direction line
        dx = math.cos(math.radians(self.angle)) * 25
        dy = math.sin(math.radians(self.angle)) * 25
        pygame.draw.line(self.screen, (0, 200, 255), (self.x, self.y), (self.x + dx, self.y + dy), 3)

        pygame.display.flip()
        self.clock.tick(30)


def main():
    rclpy.init()
    node = Driver()
    rclpy.spin(node)
    pygame.quit()
    rclpy.shutdown()
