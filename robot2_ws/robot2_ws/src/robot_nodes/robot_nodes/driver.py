import rclpy
import math
import pygame
from rclpy.node import Node
from robot_msgs.msg import Health, Command
import pymunk  #you can install pymunk in ubuntu globally by running this command :
# sudo pip install pymunk --break-system-packages

FPS = 30

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

ROBOT_SIZE = (20, 20)
DIRECTION_LINE_LENGTH = 30
ROBOT_CORNER_RADIUS = 1  #smooth round corners instead of sharp ones for better simulation

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
        self.create_timer(1 / FPS, self.render)

        # ---- PYGAME INIT ----
        pygame.init()
        pygame.display.set_caption("Robot Driver")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.clock = pygame.time.Clock()

        #pymunk / setting up the robot
        self.space = pymunk.Space()
        self.robot_body = pymunk.Body()
        self.robot_box_shape = pymunk.Poly.create_box(self.robot_body, ROBOT_SIZE, ROBOT_CORNER_RADIUS)
        self.robot_box_shape.density = 1
        self.space.add(self.robot_body, self.robot_box_shape)

        self.get_logger().info("Driver node launched")

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
    def render(self):

        # important sinon fenêtre freeze
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.quit()
                return

        self.screen.fill((15, 15, 20))

        self.draw_robot()

        self.robot_body.velocity = (15, 15)

        # direction line
        direction = pymunk.Vec2d(0, 1).rotated(self.robot_body.angle) * DIRECTION_LINE_LENGTH
        pygame.draw.line(self.screen, (0, 200, 255), tuple(self.robot_body.position),
                         tuple(self.robot_body.position + direction), 3)

        self.space.step(1 / FPS)

        pygame.display.flip()
        self.clock.tick(FPS)

    def draw_robot(self):
        world_vertices = [
            tuple(self.robot_body.position + v.rotated(self.robot_body.angle))
            for v in self.robot_box_shape.get_vertices()
        ]
        pygame.draw.polygon(self.screen, (255, 255, 255), world_vertices)

    def quit(self):
        pygame.display.quit()
        pygame.quit()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Driver()
    rclpy.spin(node)
    pygame.quit()
    rclpy.shutdown()
