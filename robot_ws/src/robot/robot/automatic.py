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

# -- Robustesse : mémoire et récupération --

# Durée de vie d'un obstacle mémorisé (s). Les obstacles récents sont conservés
# même si le lidar ne les voit plus (angle mort, latence).
OBSTACLE_MEMORY_SEC = 0.5

# Délai max sans donnée lidar avant de passer en mode conservateur (s).
LIDAR_STALE_SEC     = 0.3

# Nombre de cycles consécutifs sans trajectoire valide avant de déclencher
# la récupération.
STUCK_COUNT_THRESH  = 5

# ── Récupération par paliers ────────────────────────────────────────────────
#
#  Palier 0 – ROTATION SUR PLACE
#    Le robot est peut-être juste mal orienté. On tourne vers la direction
#    la plus dégagée sans reculer. Courte durée, peu d'énergie.
#
#  Palier 1 – RECUL + ROTATION (guidée par les obstacles)
#    On calcule le vecteur moyen des obstacles proches et on fuit dans la
#    direction opposée, avec une rotation pour s'aligner. Un bruit aléatoire
#    σ=30 % est ajouté pour ne pas répéter le même geste.
#
#  Palier 2 – RECUL AGRESSIF + ROTATION ALÉATOIRE LARGE
#    Si les deux premiers paliers ont échoué, on suppose qu'on est dans un
#    couloir ou un coin. Recul plus rapide + rotation ample tirée uniformément
#    dans [−WZ_MAX, +WZ_MAX] pour explorer des directions très différentes.
#
# Les paliers se succèdent automatiquement : chaque fois que _start_recovery
# est appelé alors qu'on sort d'un état recovery, on monte d'un palier.
# Le compteur se remet à 0 dès que DWA trouve une trajectoire valide.
# ────────────────────────────────────────────────────────────────────────────

RECOVERY_STAGES = [
    # (durée_s, vx_base, vx_noise_σ, wz_max, wz_noise_σ)
    # Palier 0 : rotation sur place, pas de recul
    dict(duration=1.0,  vx_base= 0.00, vx_sigma=0.00, wz_max=0.6,  wz_sigma=0.15),
    # Palier 1 : recul léger, rotation guidée + petit bruit
    dict(duration=1.5,  vx_base=-0.15, vx_sigma=0.04, wz_max=0.8,  wz_sigma=0.20),
    # Palier 2 : recul fort, rotation très aléatoire
    dict(duration=2.0,  vx_base=-0.25, vx_sigma=0.06, wz_max=1.0,  wz_sigma=0.30),
]


"""
Noeud Automatic - Navigation autonome avec la méthode DWA
"""

import math
import time
import random
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
        
        # ------------------------------------------------------------------ #
        # Obstacles : tableau (N, 3) → [x_local, y_local, timestamp]
        # On conserve les obstacles récents même s'ils ont disparu du scan,
        # pour combler les angles morts et la latence du lidar.
        # ------------------------------------------------------------------ #
        self._obs_memory: list[tuple[float, float, float]] = []
        self._lidar_last_stamp: float = 0.0   # horodatage du dernier scan valide

        # ------------------------------------------------------------------ #
        # Machine à états de récupération
        # États : "navigate" | "recovery"
        # ------------------------------------------------------------------ #
        self._state = "navigate"
        self._stuck_count    = 0      # nb de cycles consécutifs sans trajectoire
        self._recovery_stage = 0      # palier actuel (0, 1, 2)
        self._recovery_start: float = 0.0
        # Commande courante — recalculée chaque cycle avec bruit continu
        self._recovery_vx: float = 0.0
        self._recovery_wz: float = 0.0
        # Paramètres du palier en cours
        self._rec_vx_base:  float = 0.0
        self._rec_vx_sigma: float = 0.0
        self._rec_wz_max:   float = 0.0
        self._rec_wz_sigma: float = 0.0
        self._rec_wz_sign:  float = 1.0   # direction principale tirée au début du palier
        self._rec_duration: float = 0.0

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
                self._state = "navigate"
                self._stuck_count = 0
                self.get_logger().info("Mode MANUEL activé. Arrêt de la navigation auto.")
            else:
                self.is_auto = True
                self.goal_reached = False
                self._recovery_stage = 0
                self.get_logger().info("Mode AUTOMATIQUE activé.")
            
    def goal_callback(self, msg: Point):
        self.set_goal(msg.x, msg.y)
        
    def lidar_callback(self, msg: Lidar):
        """
        Stocke les points obstacles dans le repère ROBOT avec horodatage.
        La mémoire temporelle permet de conserver les obstacles récents
        qui ne sont plus dans le champ de vue courant (angles morts, latence).
        """
        now = time.monotonic()
        self._lidar_last_stamp = now

        # Nouveaux points du scan courant
        new_pts = [
            (d * math.cos(a), d * math.sin(a), now)
            for a, d in zip(msg.angles, msg.distances)
            if math.isfinite(d) and d > LIDAR_MIN_DIST
        ]

        # Expiration des anciens points
        cutoff = now - OBSTACLE_MEMORY_SEC
        self._obs_memory = [p for p in self._obs_memory if p[2] > cutoff]

        # Fusion : on ajoute les nouveaux points.
        # On pourrait dédupliquer mais c'est coûteux ; le ::2 ci-dessous
        # suffit à garder une densité raisonnable.
        self._obs_memory.extend(new_pts)
    
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

        # ------------------------------------------------------------------ #
        # État RECOVERY : on exécute la manœuvre avec bruit continu
        # ------------------------------------------------------------------ #
        if self._state == "recovery":
            elapsed = time.monotonic() - self._recovery_start
            if elapsed < self._rec_duration:
                # Bruit gaussien re-tiré à chaque cycle pour un mouvement
                # organique — le robot "tâtonne" plutôt que de tracer
                # une trajectoire rigide.
                vx_noisy = self._rec_vx_base + random.gauss(0.0, self._rec_vx_sigma)
                wz_noisy = (self._rec_wz_sign * self._rec_wz_max
                            + random.gauss(0.0, self._rec_wz_sigma))
                wz_noisy = _clamp(wz_noisy,
                                  -RECOVERY_STAGES[-1]["wz_max"],
                                   RECOVERY_STAGES[-1]["wz_max"])
                self._publish_command(vx_noisy, 0.0, wz_noisy)
                return
            else:
                # Fin du palier → retour à la navigation normale
                self._state = "navigate"
                self._stuck_count = 0
                self.get_logger().info(
                    f"Récupération palier {self._recovery_stage} terminée — "
                    "reprise de la navigation."
                )

        # ------------------------------------------------------------------ #
        # Avertissement lidar périmé
        # ------------------------------------------------------------------ #
        now = time.monotonic()
        lidar_age = now - self._lidar_last_stamp
        if self._lidar_last_stamp > 0 and lidar_age > LIDAR_STALE_SEC:
            self.get_logger().warn(
                f"Lidar périmé ({lidar_age:.2f}s) — navigation conservatrice"
            )
        
        # ------------------------------------------------------------------ #
        # Vérification de l'atteinte de la cible
        # ------------------------------------------------------------------ #
        dist_to_goal = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
        if dist_to_goal < NAV_GOAL_DIST_TOL:
            self.get_logger().info("Cible atteinte !")
            self.goal_reached = True
            self._stop()
            return
        
        # ------------------------------------------------------------------ #
        # Calcul du but dans le repère robot
        # ------------------------------------------------------------------ #
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        cos_t, sin_t = math.cos(self.robot_theta), math.sin(self.robot_theta)
        goal_local_x =  dx * cos_t + dy * sin_t
        goal_local_y = -dx * sin_t + dy * cos_t

        # ------------------------------------------------------------------ #
        # Construction du nuage d'obstacles (avec mémoire + sous-éch. léger)
        # ------------------------------------------------------------------ #
        obs = self._get_obstacle_array()

        # ------------------------------------------------------------------ #
        # Vérification : est-on déjà trop proche d'un obstacle ?
        # ------------------------------------------------------------------ #
        if obs.shape[0] > 0:
            min_dist_now = float(np.hypot(obs[:, 0], obs[:, 1]).min())
            if min_dist_now < DWA_ROBOT_RADIUS:
                self.get_logger().warn(
                    f"Obstacle à {min_dist_now:.3f}m — déclenchement immédiat de la récupération"
                )
                self._start_recovery()
                return

        # ------------------------------------------------------------------ #
        # Calcul DWA
        # ------------------------------------------------------------------ #
        best_cmd, _ = self._dwa(goal_local_x, goal_local_y, obs)
        
        if best_cmd is None:
            self._stuck_count += 1
            self.get_logger().warn(
                f"DWA : aucune trajectoire sûre ({self._stuck_count}/{STUCK_COUNT_THRESH})"
            )
            self._stop()

            if self._stuck_count >= STUCK_COUNT_THRESH:
                self._start_recovery()
        else:
            self._stuck_count = 0
            self._recovery_stage = 0   # succès → on repart du palier 0
            vx, vy, wz = best_cmd
            self._publish_command(vx, vy, wz)

    # ====================================================================== #
    #  Récupération par paliers avec direction de fuite guidée
    # ====================================================================== #

    def _start_recovery(self):
        """
        Déclenche (ou escalade) la manœuvre de récupération.

        Stratégie :
        ─────────────────────────────────────────────────────────────────────
        1. On calcule la direction de fuite optimale à partir des obstacles
           proches : vecteur répulsif = moyenne des vecteurs (robot→obstacle)
           inversée. C'est la direction la plus dégagée.

        2. On ajoute un bruit gaussien autour de cette direction. Le bruit
           augmente avec le palier, ce qui élargit l'espace exploré si les
           tentatives précédentes ont échoué.

        3. Le signe de la rotation est cohérent avec la direction de fuite
           (on tourne vers l'espace libre), mais légèrement bruité.

        Paliers successifs (voir RECOVERY_STAGES) :
          0 → rotation sur place (on cherche juste un meilleur cap)
          1 → recul modéré + rotation guidée
          2 → recul fort + rotation très aléatoire (exploration large)
        ─────────────────────────────────────────────────────────────────────
        """
        # Escalade du palier si on était déjà en recovery
        if self._state == "recovery":
            self._recovery_stage = min(
                self._recovery_stage + 1, len(RECOVERY_STAGES) - 1
            )
        # Sinon on repart du palier 0 sauf si on enchaîne des échecs
        # (stuck_count >> thresh : on saute directement au palier 1)
        elif self._stuck_count >= STUCK_COUNT_THRESH * 3:
            self._recovery_stage = min(
                self._recovery_stage + 1, len(RECOVERY_STAGES) - 1
            )
        else:
            # On ne remet PAS à zéro le palier entre deux blocages proches
            pass

        stage = RECOVERY_STAGES[self._recovery_stage]
        self._rec_vx_base  = stage["vx_base"]
        self._rec_vx_sigma = stage["vx_sigma"]
        self._rec_wz_max   = stage["wz_max"]
        self._rec_wz_sigma = stage["wz_sigma"]
        self._rec_duration = stage["duration"]

        # ── Direction de fuite ──────────────────────────────────────────── #
        # Vecteur répulsif : somme des vecteurs pointant DU robot VERS
        # chaque obstacle, retournée (on veut s'éloigner de la masse).
        obs = self._get_obstacle_array()
        escape_angle = 0.0  # repère robot → 0 = tout droit devant

        if obs.shape[0] > 0:
            # On ne considère que les obstacles proches pour la direction
            dists = np.hypot(obs[:, 0], obs[:, 1])
            nearby = obs[dists < (DWA_ROBOT_RADIUS + DWA_OBSTACLE_MARGIN) * 4]
            if nearby.shape[0] > 0:
                # Vecteur répulsif pondéré par 1/d² (les plus proches comptent plus)
                d2 = np.hypot(nearby[:, 0], nearby[:, 1]) ** 2 + 1e-6
                rep_x = -np.sum(nearby[:, 0] / d2)
                rep_y = -np.sum(nearby[:, 1] / d2)
                escape_angle = math.atan2(rep_y, rep_x)

        # Signe de rotation : on tourne vers l'angle de fuite
        # (si escape_angle > 0 → obstacle à gauche → on tourne à droite, etc.)
        self._rec_wz_sign = -math.copysign(1.0, escape_angle) if abs(escape_angle) > 0.1 \
                             else random.choice([-1.0, 1.0])

        self._state = "recovery"
        self._recovery_start = time.monotonic()
        self._stuck_count = 0

        self.get_logger().warn(
            f"RECOVERY palier {self._recovery_stage} | "
            f"vx_base={self._rec_vx_base:.2f} m/s  "
            f"wz_max={self._rec_wz_max:.2f} rad/s  "
            f"escape_angle={math.degrees(escape_angle):.0f}°  "
            f"durée={self._rec_duration:.1f}s"
        )

    # ====================================================================== #
    #  Gestion des obstacles avec mémoire
    # ====================================================================== #

    def _get_obstacle_array(self) -> np.ndarray:
        """
        Retourne un tableau (N, 2) des positions d'obstacles (repère robot)
        en fusionnant le scan courant et la mémoire temporelle.
        
        On prend un point sur deux pour éviter une explosion du coût de
        _simulate_trajectory, mais SANS sauter les points proches (on applique
        le sous-échantillonnage uniquement aux points éloignés).
        """
        if not self._obs_memory:
            return np.empty((0, 2), dtype=np.float64)

        pts = np.array([(x, y) for x, y, _ in self._obs_memory], dtype=np.float64)

        # Séparer les points proches (< 2×rayon de sécurité) des points lointains
        dists = np.hypot(pts[:, 0], pts[:, 1])
        close_mask = dists < (DWA_ROBOT_RADIUS + DWA_OBSTACLE_MARGIN) * 2.0
        close_pts  = pts[close_mask]
        far_pts    = pts[~close_mask][::2]  # sous-éch. ×2 seulement pour les lointains

        return np.vstack([close_pts, far_pts]) if (close_pts.shape[0] + far_pts.shape[0]) > 0 \
               else np.empty((0, 2), dtype=np.float64)

    # ====================================================================== #
    #  Algorithme DWA
    # ====================================================================== #
    
    def _dwa(self, goal_lx: float, goal_ly: float, obs: np.ndarray):
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
        self._state = "navigate"
        self._stuck_count = 0
        self._recovery_stage = 0
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