import rclpy
import math
import pygame
from rclpy.node import Node
from msgs.msg import Health, WheelSpeeds
import pymunk
from simulation.robot import Robot

FPS = 30
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600


class Simulation(Node):

    def __init__(self):
        super().__init__("driver")

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)

        self.create_subscription(WheelSpeeds, "/robot/wheels", self.wheels_callback, 1)

        #self.create_timer(0.1, self.heartbeat)

        self.space = pymunk.Space()
        self.space.damping = 0

        self.robot = Robot(self.space)
        self.get_logger().info("Driver node launched")

    def heartbeat(self):
        self.pub_health.publish(Health(state="Hello", name="driver"))

    def wheels_callback(self, msg: WheelSpeeds):
        self.get_logger().info(
            f"received speeds : {msg.wheel1_speed}, {msg.wheel2_speed}, {msg.wheel3_speed}, {msg.wheel4_speed}")
        self.robot.update_wheel_speeds(
            msg.wheel1_speed * 100,
            msg.wheel2_speed * 100,
            msg.wheel3_speed * 100,
            msg.wheel4_speed * 100,
        )


def main():
    rclpy.init()
    node = Simulation()

    # ---- PYGAME INIT ----
    pygame.init()
    pygame.display.set_caption("Robot Driver")
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()

    running = True

    # The Master Loop
    while running and rclpy.ok():
        # 1. Process ROS callbacks (timeout_sec=0 means "don't block, just check and move on")
        rclpy.spin_once(node, timeout_sec=0.0)

        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Step Physics
        node.robot.update(1 / FPS)
        node.space.step(1 / FPS)

        # Render
        screen.fill((15, 15, 20))
        node.robot.draw(screen)
        pygame.display.flip()

        # Lock Framerate
        clock.tick(FPS)

    print("Shutting down...")
    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
