"""
Noeud Automatic - Navigation autonome avec la méthode APF qui est pour l'instant une piste

S'abonne à :
    - /robot/lidar_obstacles (msgs/Lidar) : points d'obstacles filtrés par le Lidar
    - /robot/pos : position et orientation courante du robot

Publie sur :
    - /robot/command : ordres de vitesse vers le noeud control
"""

import math
from robot.steady_node import SteadyNode
import rclpy
from msgs.msgs import Lidar, Command
from geometry_msgs.msg import Pose2D

#-- Paramètres à ajuster --

# Cible à atteindre (en mètres dans le repère de départ)
GOAL_X = 1.0
GOAL_Y = 0.0

# Champ attractif
K_ATT = 1.5 # gain d'attraction
GOAL_RADIUS = # distance en mètres en-dessous de laquelle la cible est considérée comme atteinte

# Champ répulsif
K_REP = 0.8 # gain répulsif
D0 = 0.5 # rayon d'influence des obstacles en mètres : DOIT correspondre à obstacle_range du noeud lidar
MIN_DIST = 0.05 # distance plancher pour pas diviser par 0

# Conversion force -> consigne PWM
MAX_SPEED = 200 # valeur PWM max envoyée au noeud control (entier, même echelle que la console)
MAX_LINEAR_FORCE = 3.0
MAX_ANGULAR_FORCE = 2.0

# Rayon approximatif du robot (m)
ROBOT_RADIUS = 0.15

# Fréquance de la boucle de contrôle
PUBLISH_HZ = 10.0

class Automatic(SteadyNode):
    
    def __init__(self):
        super().__init__('automatic')
        
        # Abonnements
        self.create_subscription(
            Lidar, '/robot/lidar_obstacles', self.lidar_callback, 1
        )
        self.create_subscription(
            Pose2D, '/robot/pos', self.pos_callback, 10
        )
        
        # Publication vers le noeud control
        self.pub_cmd = self.create_publisher(Command, '/robot/command', 10)
        
        # Etat
        self.obstacle_angles: list[float] = []
        self.obstacle_distances = list[float] = []
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_theta: float = 0.0
        
        self.goal_x: float = GOAL_X
        self.goal_y: float = GOAL_Y
        self.goal_reached = False
        
        # BOucle de controle d'arrivée à destination (fréquance fixe)
        self.create_timer(1.0 / PUBLISH_HZ, self.control_loop)
        
        self.get_logger().info(
            f"Noeud Automatic lancé - cible : ({self.goal_x:.2f},{self.goal_y:.2f})"
        )
        
    #-- Callbacks --
    
    def lidar_callback(self, msg:Lidar):
        self.obstacle_angles = list(msg.angles)
        self.obstacle_distances = list(msg.distances)
    
    def pos_callback(self, msg:Pose2D):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta
    
    
    #-- Boucle principale --
    
    def control_loop(self):
        if self.goal_reached:
            self._stop()
            return
        
        # Verification de l'atteinte de la cible
        dx_goal = self.goal_x - self.robot_x
        dy_goal = self.goal_y - self.robot_y
        if math.hypot(dx_goal, dy_goal) < GOAL_RADIUS:
            self.get_logger().info("Cible atteinte !")
            self.goal_reached = True
            self._stop()
            return
        
        
        # Clacul de la force APF
        fx, fy = self._compute_apf(dx_goal, dy_goal)
        
        # Conversion en consignes roues et publication
        vx, vy, w = self._force_to_velocity(fx, fy)
        self._publish_command(vx, vy, w)
        
        #-- APF --
        def _compute_apf(self, dx_goal: float, dy_goal: float):
            """
            Calcule la force totale (fx, fy) dans le repère 'monde', celui du départ du robot
            """
            pass
        
        #-- Conversion force -> vitesse --
        def _force_to_velocity(self, fx: float, fy: float):
            """
            Projette la force dans le repère robot et calcule la vitesse angulaire pour aligner le cap.
            """
            pass
        
        #-- Publication -- 
        def _publish_command(self, vx: float, vy: float, w: float):
            """
            Convertit (vx, vy, w) en 4 vitesses PWM via la cinématique et publie un Command vers le
            topic /robot/command.
            
            Convention des arguements (identique à la console):
                arg1 = avant-gauche, arg2 = avant-droite,
                arg3 = arrière-droite, arg4 = arrière-gauche
            """
            pass
        
        def _stop(self):
            cmd = Command()
            cmd.action = 'speed'
            cmd.arg1 = cmd.arg2 = cmd.arg3 = cmd.arg4 = 0
            self.pub_cmd.publish(cmd)
            
        #-- Public -- 
        def set_goal(self, x: float, y: float):
            self.goal_x = x
            self.goal_y = y
            self.goal_reached = False
            self.get_logger().info(f"Nouvelle cible : ({x:.2f},{y:.2f})")
        

#-- Point d'entrée --

def main():
    rclpy.init()
    node = Automatic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()