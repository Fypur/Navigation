import rclpy
import math
import pygame
from rclpy.node import Node
from msgs.msg import Health, WheelSpeeds, Lidar, RPMs
import pymunk
from simulation.robot import Robot

FPS = 10
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Constante de conversion (100 pixels = 1 mètre)
PIXELS_PER_METER = 100.0

class Simulation(Node):

    def __init__(self):
        super().__init__("driver")

        self.pub_health = self.create_publisher(Health, "/robot/health", 10)
        self.create_subscription(WheelSpeeds, "/robot/wheels", self.wheels_callback, 1)
        
        # -- Nouveaux publishers --
        self.pub_lidar = self.create_publisher(Lidar, '/robot/lidar', 10)
        self.pub_encoders = self.create_publisher(RPMs, '/robot/encoders', 10)
        
        # Timer pour publier les capteurs à 10 Hz
        self.create_timer(0.1, self.publish_sensors)
        
        #self.create_timer(0.1, self.heartbeat)

        self.space = pymunk.Space()
        self.space.damping = 0
        
        # Création de la carte
        self.walls = self.create_map()

        self.robot = Robot(self.space)
        self.robot.body.position = (SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2) # positionner au milieu
        self.get_logger().info("Driver node launched with Map and Lidar")

    def create_map(self):
        static_body = self.space.static_body
        walls_segments = [
            pymunk.Segment(static_body, (50, 50), (750, 50), 3),     # Haut
            pymunk.Segment(static_body, (750, 50), (750, 550), 3),   # Droite
            pymunk.Segment(static_body, (750, 550), (50, 550), 3),   # Bas
            pymunk.Segment(static_body, (50, 550), (50, 50), 3),     # Gauche
            pymunk.Segment(static_body, (200, 200), (300, 400), 5),  # Obstacle central
            pymunk.Segment(static_body, (500, 400), (600, 400), 5)   # Obstacle central 2
        ]
        for wall in walls_segments:
            wall.elasticity = 0.5
            wall.fricition = 0.5
            # On palce les murs dans la catégorie 2
            wall.filter = pymunk.ShapeFilter(categories=0b10)
        
        self.space.add(*walls_segments)
        return walls_segments
    

    def wheels_callback(self, msg: WheelSpeeds):
        self.get_logger().info(
            f"Received speeds : {msg.front_left_wheel_speed}, {msg.front_right_wheel_speed}, {msg.back_right_wheel_speed}, {msg.back_left_wheel_speed}"
        )
        # On sauvegarde les valeurs pour les renvoyer dans l'encodeur plus tard
        self.current_speeds = msg 
        self.robot.update_wheel_speeds(
            msg.front_left_wheel_speed * 100,
            msg.front_right_wheel_speed * 100,
            msg.back_right_wheel_speed * 100,
            msg.back_left_wheel_speed * 100,
        )
    
    # -- Simule et publie les données du Lidar et les Encodeurs --
    def publish_sensors(self):
        
        # -- Heartbeat --
        self.pub_health.publish(Health(state="OK", name="driver"))
        
        
        # -- Simulation du Lidar --
        lidar_msg = Lidar()
        max_range_px = 300.0 # Portée de 3 mètres (300 pixels)
        
        # On veut détecter les collisions avec les murs (catégorie 2)
        lidar_filter = pymunk.ShapeFilter(mask=0b10)
        
        robot_pos = self.robot.body.position
        robot_angle = self.robot.body.angle
        
        # On tire 1 rayon tous 5 les degrès)
        self.hit_points = []
        for i in range (72): # 360/5 = 72 points
            # Angle du rayon = angle du robot + angle de tir
            local_angle = math.radians(i)
            global_angle = robot_angle + local_angle
            
            end_point = robot_pos + pymunk.Vec2d(max_range_px, 0).rotated(global_angle)
            
            # Pymunk calcule l'intersection du rayon avec les murs
            query = self.space.segment_query_first(robot_pos, end_point, 1, lidar_filter)
            
            if query:
                # Mur touché
                dist_px = robot_pos.get_distance(query.point)
                self.hit_points.append(query.point)
            else:
                # Rien touché
                dist_px = 1000.0
                self.hit_points.append(end_point)
        
            # Ajout au message (conversion des pixels en metres)
            lidar_msg.angles.append(float(local_angle))
            lidar_msg.distances.append(float(dist_px / PIXELS_PER_METER))
        
        self.pub_lidar.publish(lidar_msg)
        
        # -- Simulation des Encodeurs --
        rpm_msg = RPMs()
        try:
            rpm_msg.front_left_rpm = float(self.robot.wheel1.speed)
            rpm_msg.front_right_rpm = float(self.robot.wheel2.speed)
            rpm_msg.back_left_rpm = float(self.robot.wheel3.speed)
            rpm_msg.back_right_rpm = float(self.robot.wheel4.speed)
        except AttributeError:
            pass

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
        
        for wall in node.walls:
            pygame.draw.line(screen, (200, 200, 200), wall.a, wall.b, int(wall.radius * 2))
        
        if hasattr(node, 'hit_points'):
            for point in node.hit_points:
                pygame.draw.line(screen, (0, 50, 0), node.robot.body.position, point, 1) # Rayon
                pygame.draw.circle(screen, (0, 255, 0), (int(point[0]), int(point[1])), 3) # Impact
                
        
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
