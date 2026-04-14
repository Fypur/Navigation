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
from msgs.msg import Lidar, Command
from geometry_msgs.msg import Pose2D

#-- Paramètres à ajuster --

# Cible à atteindre (en mètres dans le repère de départ)
GOAL_X = 1.0
GOAL_Y = 0.0

# Champ attractif
K_ATT = 1.5 # gain d'attraction
GOAL_RADIUS = 0.1 # distance en mètres en-dessous de laquelle la cible est considérée comme atteinte

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
        self.obstacle_distances: list[float] = []
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
        dist_goal = math.hypot(dx_goal, dy_goal)
        if dist_goal > 0 :
            fx_att = K_ATT * dx_goal / dist_goal
            fy_att = K_ATT * dy_goal / dist_goal
        else:
            fx_att = 0
            fy_att = 0

        fx_rep, fy_rep = 0.0, 0.0
        for angle, d in zip(self.obstacle_angles, self.obstacle_distances):
            if not math.isfinite(d) or d < MIN_DIST or d > D0:
                continue
            d = max(d, MIN_DIST) # éviter les divisions par 0
            
            # Direction obstacle dans le repère monde
            angle_world = self.robot_theta + angle
            ox = d * math.cos(angle_world)
            oy = d * math.sin(angle_world)
            coeff = K_REP * (1.0 / d - 1.0 / D0) / (d**2)
            # Force opposée à la direction robot -> obstacle
            fx_rep -= coeff * ox / d
            fy_rep -= coeff * oy / d
        
        return fx_att + fx_rep, fy_att + fy_rep
        
        
    #-- Conversion force -> vitesse --
    def _force_to_velocity(self, fx: float, fy: float):
        """
        Projette la force dans le repère robot et calcule la vitesse angulaire pour aligner le cap.
        """
        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)
        
        # Projection dans le repère robot
        vx = cos_t * fx + sin_t * fy
        vy = -sin_t * fx + cos_t * fy
        
        # COrrection angulaire pour aligner l robot sur la direction de la force
        desired_heading = math.atan2(fy, fx)
        heading_error = _angle_wrap(desired_heading - self.robot_theta)
        w = 1.0 * heading_error # gain angulaire (à ajuster)
        
        return vx, vy, w
            
        
    #-- Publication -- 
    def _publish_command(self, vx: float, vy: float, w: float):
        """
        Convertit (vx, vy, w) en 4 vitesses PWM via la cinématique et publie un Command vers le
        topic /robot/command.
        
        Convention des arguements (identique à la console):
            arg1 = avant-gauche, arg2 = avant-droite,
            arg3 = arrière-droite, arg4 = arrière-gauche
        """
        
        # Normalisation de la vitesse
        vx_n = _clamp(vx / MAX_LINEAR_FORCE, -1.0, 1.0)
        vy_n = _clamp(vy / MAX_LINEAR_FORCE, -1.0, 1.0)
        w_n = _clamp(w / MAX_ANGULAR_FORCE, -1.0, 1.0)
        
        # Modèle cinématique pour un robot à 4 roues toutes directions
        fl = vx_n - vy_n - w_n # avant-gauche
        fr = vx_n + vy_n + w_n # avant-droite
        rr = vx_n - vy_n + w_n # arrière-droite
        rl = vx_n + vy_n - w_n # arrière-gauche
        
        # Normalisation pour que la valeur max soit 1.0
        max_val = max(abs(fl), abs(fr), abs(rr), abs(rl), 1.0)
        fl /= max_val
        fr /= max_val
        rr /= max_val
        rl /= max_val
        
        def to_pwm(v: float):
            return int(v * MAX_SPEED)
        
        cmd = Command()
        cmd.action = 'speed'
        cmd.arg1 = to_pwm(fl)
        cmd.arg2 = to_pwm(fr)
        cmd.arg3 = to_pwm(rr)
        cmd.arg4 = to_pwm(rl)
        
        self.pub_cmd.publish(cmd)
        self.get_logger().debug(
            f"APF -> FL={cmd.arg1}, FR={cmd.arg2}, RR={cmd.arg3}, RL={cmd.arg4}"
        )
    
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
        
        
#-- Fonctions utilitaires --

def _angle_wrap(angle: float) -> float:
    """Ramène un angle à l'intervalle [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def _clamp(v: float, lo: float, hi: float) -> float:
    """Limite v à l'intervalle [lo, hi]"""
    return max(lo, min(hi, v))


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