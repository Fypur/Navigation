# -- Navigation DWA --
DWA_MAX_VX          = 0.5     # m/s   vitesse longitudinale max
DWA_MAX_VY          = 0.5     # m/s   vitesse latérale max
DWA_MAX_WZ          = 1.0     # rad/s vitesse angulaire max
DWA_MAX_ACC_V       = 1.0     # m/s²  accélération linéaire max
DWA_MAX_ACC_W       = 2.0     # rad/s² accélération angulaire max

DWA_V_SAMPLES       = 5       # nombre d'échantillons sur vx (et vy)
DWA_W_SAMPLES       = 7       # nombre d'échantillons sur wz

DWA_SIM_TIME        = 1.5     # s     durée de simulation d'une trajectoire
DWA_SIM_STEPS       = 15      # int   pas de simulation

DWA_ROBOT_RADIUS    = 0.15    # m     rayon du robot (clearance)
DWA_OBSTACLE_MARGIN = 0.15    # m     marge de sécurité supplémentaire
DWA_LETHAL_DIST     = 0.10    # m     distance en dessous de laquelle on bloque

DWA_W_HEADING       = 0.5     # poids cap vers le but
DWA_W_CLEARANCE     = 0.2     # poids distance aux obstacles
DWA_W_VELOCITY      = 0.1     # poids vitesse (favorise les trajectoires rapides)
DWA_W_GOAL_DIST     = 0.2     # poids distance euclidienne au but

NAV_GOAL_DIST_TOL   = 0.2     # m     on considère le but atteint
NAV_GOAL_ANGLE_TOL  = 0.2     # rad   tolérance angulaire

"""
Noeud Automatic - Navigation autonome avec la méthode DWA
"""

import math
import numpy as np
import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import Lidar, RPMs
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Bool

from robot.robot_config import (
    MAX_RPM, MIN_RPM, LIDAR_MIN_DIST, LIDAR_MAX_RANGE_M,
    DEFAULT_GOAL_X, DEFAULT_GOAL_Y, AUTO_LOOP_HZ
)


def _angle_wrap(angle: float) -> float:
    """Ramène un angle à l'intervalle [-pi, pi]"""
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

def _clamp(v: float, lo: float, hi: float) -> float:
    """Limite v à l'intervalle [lo, hi]"""
    return max(lo, min(hi, v))


class Automatic(SteadyNode):
    
    def __init__(self):
        super().__init__('automatic')
        
        # -- Abonnements --
        self.create_subscription(Lidar, '/robot/lidar_obstacles', self.lidar_callback, 1)
        self.create_subscription(Pose2D, '/robot/pos', self.pos_callback, 10)
        self.create_subscription(Point, '/robot/automatic_goal', self.goal_callback, 10)
        self.create_subscription(Bool, '/robot/enable_auto', self.enable_auto_callback, 10)
        
        # -- Publication vers le noeud control --
        self.pub_cmd = self.create_publisher(RPMs, '/robot/command', 10)
        
        # -- Etat --
        self.is_auto = False
        self.goal_reached = False
        
        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_theta: float = 0.0
        
        # Vitesses courantes (mémorisées pour la fenêtre dynamique)
        self.vx_cur = 0.0
        self.vy_cur = 0.0
        self.wz_cur = 0.0
        
        self.goal_x: float = DEFAULT_GOAL_X
        self.goal_y: float = DEFAULT_GOAL_Y
        
        # Obstacles en repère local
        self.obstacle_pts = np.empty((0, 2), dtype=np.float64)
        
        # Boucle de contrôle
        self.create_timer(1.0 / AUTO_LOOP_HZ, self.control_loop)
        
        self.get_logger().info(
            f"Noeud Automatic (DWA) lancé - cible : ({self.goal_x:.2f},{self.goal_y:.2f})"
        )
        
    # -- Callbacks --
    
    def enable_auto_callback(self, msg: Bool):
        if self.is_auto != msg.data:
            if self.is_auto:
                self._stop()
                self.is_auto = False
                self.get_logger().info("Mode MANUEL activé. Arrêt de la navigation auto.")
            else:
                self.is_auto = True
                self.goal_reached = False
                self.get_logger().info("Mode AUTOMATIQUE activé.")
            
    def goal_callback(self, msg: Point):
        self.set_goal(msg.x, msg.y)
        
    def lidar_callback(self, msg: Lidar):
        """Stocke les points obstacles projetés dans le repère ROBOT."""
        pts = [
            [d * math.cos(a), d * math.sin(a)] 
            for a, d in zip(msg.angles, msg.distances)
            if math.isfinite(d) and d > LIDAR_MIN_DIST
        ]
        self.obstacle_pts = np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))
    
    def pos_callback(self, msg: Pose2D):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta
    
    
    # -- Boucle principale --
    
    def control_loop(self):
        if not self.is_auto:
            return
        
        if self.goal_reached:
            self._stop()
            return
        
        # Vérification de l'atteinte de la cible
        dist_to_goal = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
        if dist_to_goal < NAV_GOAL_DIST_TOL:
            self.get_logger().info("Cible atteinte !")
            self.goal_reached = True
            self._stop()
            return
        
        # Calcul du but dans le repère robot
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        cos_t, sin_t = math.cos(self.robot_theta), math.sin(self.robot_theta)
        goal_local_x =  dx * cos_t + dy * sin_t
        goal_local_y = -dx * sin_t + dy * cos_t

        # Calcul DWA
        best_cmd, best_score = self._dwa(goal_local_x, goal_local_y)
        
        if best_cmd is None:
            self.get_logger().warn("DWA : aucune trajectoire sûre — arrêt d'urgence")
            self._stop()
        else:
            vx, vy, wz = best_cmd
            self._publish_command(vx, vy, wz)


    # -- Algorithme DWA --
    
    def _dwa(self, goal_lx: float, goal_ly: float):
        dt_ctrl = 1.0 / AUTO_LOOP_HZ
        acc_v   = DWA_MAX_ACC_V * dt_ctrl
        acc_w   = DWA_MAX_ACC_W * dt_ctrl

        vx_min = max(-DWA_MAX_VX, self.vx_cur - acc_v)
        vx_max = min( DWA_MAX_VX, self.vx_cur + acc_v)
        vy_min = max(-DWA_MAX_VY, self.vy_cur - acc_v)
        vy_max = min( DWA_MAX_VY, self.vy_cur + acc_v)
        wz_min = max(-DWA_MAX_WZ, self.wz_cur - acc_w)
        wz_max = min( DWA_MAX_WZ, self.wz_cur + acc_w)

        vx_arr = np.linspace(vx_min, vx_max, DWA_V_SAMPLES)
        vy_arr = np.linspace(vy_min, vy_max, DWA_V_SAMPLES)
        wz_arr = np.linspace(wz_min, wz_max, DWA_W_SAMPLES)

        best_score = -np.inf
        best_cmd   = None
        obs = self.obstacle_pts[::5]

        for vx in vx_arr:
            for vy in vy_arr:
                for wz in wz_arr:
                    min_clearance, final_x, final_y, final_theta = \
                        self._simulate_trajectory(vx, vy, wz, obs)

                    # Collision
                    if min_clearance < DWA_ROBOT_RADIUS + DWA_OBSTACLE_MARGIN:
                        continue

                    # Heading
                    goal_angle = math.atan2(goal_ly, goal_lx)
                    final_angle_to_goal = _angle_wrap(goal_angle - final_theta)
                    heading_score = (1.0 + math.cos(final_angle_to_goal)) / 2.0

                    # Clearance
                    clearance_score = min(min_clearance, LIDAR_MAX_RANGE_M) / LIDAR_MAX_RANGE_M

                    # Velocity
                    v_norm = math.hypot(vx, vy) / math.hypot(DWA_MAX_VX, DWA_MAX_VY)
                    velocity_score = v_norm

                    # Goal dist
                    final_dist = math.hypot(goal_lx - final_x, goal_ly - final_y)
                    max_possible = math.hypot(goal_lx, goal_ly) + DWA_MAX_VX * DWA_SIM_TIME
                    goal_dist_score = 1.0 - min(final_dist / max(max_possible, 1e-6), 1.0)

                    score = (DWA_W_HEADING   * heading_score
                           + DWA_W_CLEARANCE * clearance_score
                           + DWA_W_VELOCITY  * velocity_score
                           + DWA_W_GOAL_DIST * goal_dist_score)

                    if score > best_score:
                        best_score = score
                        best_cmd   = (vx, vy, wz)

        # Mémorisation
        if best_cmd is not None:
            self.vx_cur, self.vy_cur, self.wz_cur = best_cmd
        else:
            self.vx_cur, self.vy_cur, self.wz_cur = 0.0, 0.0, 0.0

        return best_cmd, best_score

    def _simulate_trajectory(self, vx: float, vy: float, wz: float, obs: np.ndarray):
        dt   = DWA_SIM_TIME / DWA_SIM_STEPS
        x, y, theta = 0.0, 0.0, 0.0
        min_clearance = np.inf

        for _ in range(DWA_SIM_STEPS):
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            x     += (vx * cos_t - vy * sin_t) * dt
            y     += (vx * sin_t + vy * cos_t) * dt
            theta  = _angle_wrap(theta + wz * dt)

            if obs.shape[0] > 0:
                dx_obs = obs[:, 0] - x
                dy_obs = obs[:, 1] - y
                dists  = np.hypot(dx_obs, dy_obs)
                min_d  = float(dists.min())
                if min_d < min_clearance:
                    min_clearance = min_d

                if min_d < DWA_LETHAL_DIST:
                    return min_d, x, y, theta

        if obs.shape[0] == 0:
            min_clearance = LIDAR_MAX_RANGE_M

        return min_clearance, x, y, theta
            
        
    # -- Publication -- 
    def _publish_command(self, vx: float, vy: float, w: float):
        """
        Convertit (vx, vy, w) généré par DWA en vitesses PWM et publie un RPMs.
        """
        # Normalisation pour le modèle cinématique
        vx_n = _clamp(vx / DWA_MAX_VX, -1.0, 1.0)
        vy_n = _clamp(vy / DWA_MAX_VY, -1.0, 1.0)
        w_n  = _clamp(w / DWA_MAX_WZ, -1.0, 1.0)
        
        # Modèle cinématique
        fl = vx_n - vy_n - w_n 
        fr = vx_n + vy_n + w_n 
        rr = vx_n - vy_n + w_n 
        rl = vx_n + vy_n - w_n 
        
        max_val = max(abs(fl), abs(fr), abs(rr), abs(rl), 1.0)
        fl /= max_val
        fr /= max_val
        rr /= max_val
        rl /= max_val
            
        def to_real_rpm(norm_val: float) -> float:
            if abs(norm_val) < 0.02: 
                return 0.0
            vitesse_reelle = MIN_RPM + (MAX_RPM - MIN_RPM) * abs(norm_val)
            return math.copysign(vitesse_reelle, norm_val)
        
        cmd = RPMs()
        cmd.front_left_rpm  = float(to_real_rpm(fl))
        cmd.front_right_rpm = float(to_real_rpm(fr))
        cmd.back_right_rpm  = float(to_real_rpm(rr))
        cmd.back_left_rpm   = float(to_real_rpm(rl))
 
        self.pub_cmd.publish(cmd)
    
    def _stop(self):
        self.vx_cur, self.vy_cur, self.wz_cur = 0.0, 0.0, 0.0
        cmd = RPMs()
        cmd.front_left_rpm = cmd.front_right_rpm = cmd.back_right_rpm = cmd.back_left_rpm = 0.0
        self.pub_cmd.publish(cmd)
        
    # -- Public -- 
    def set_goal(self, x: float, y: float):
        self.goal_x = x
        self.goal_y = y
        self.goal_reached = False
        self.get_logger().info(f"Nouvelle cible (DWA) : ({x:.2f},{y:.2f})")


# -- Point d'entrée --
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