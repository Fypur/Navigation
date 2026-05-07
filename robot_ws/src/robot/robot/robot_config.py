import math


# PARAMÈTRES PHYSIQUES DU ROBOT

WHEEL_RADIUS = 0.04     # Rayon des roues en mètres (4 cm)
ROBOT_RADIUS = 0.15     # Rayon approximatif du robot pour l'APF (m)
LX = 0.15               # Distance centre -> axe avant/arrière (m)
LY = 0.15               # Distance centre -> axe gauche/droite (m)
GEAR_RATIO = 1.0        # Rapport de réduction (1.0 = RPM moteur == RPM roue)

ROBOT_SIZE = 0.80       # Taille du robot pour la détection de collision (m)


# PARAMÈTRES DES ACTIONNEURS

MAX_RPM = 160.0         # RPM maximum envoyé aux roues (même échelle que la console)


# PARAMÈTRES DU LIDAR


LIDAR_MAX_RANGE_M   = 6.0   # Portée maximale du lidar (m)
LIDAR_N_RAYS        = 360   # Nombre de rayons / Simulation
LIDAR_MIN_DIST      = 0.05  # Distance plancher pour ignorer les points trop proches (m)
LIDAR_OBSTACLE_RANGE = 2.0  # Distance en dessous de laquelle un point est un "obstacle" (m)


# PARAMÈTRES DE NAVIGATION AUTOMATIQUE (APF)


K_ATT           = 1.5   # Gain du champ attractif
K_REP           = 0.8   # Gain du champ répulsif
APF_D0          = 0.5   # Rayon d'influence des obstacles (m) — doit correspondre à LIDAR_OBSTACLE_RANGE
APF_MIN_DIST    = 0.05  # Distance plancher pour éviter la division par zéro (m)
GOAL_RADIUS     = 0.2   # Distance en dessous de laquelle la cible est considérée atteinte (m)


# PARAMÈTRES DE SIMULATION


FPS             = 30            # Fréquence de rendu pygame (Hz)
FREQ_PUB        = 0.1           # Période de publication des capteurs simulés (s) → 10 Hz
SCREEN_WIDTH    = 1200          # Largeur de la fenêtre de simulation (pixels)
SCREEN_HEIGHT   = 800           # Hauteur de la fenêtre de simulation (pixels)
PIXELS_PER_METER = 100.0        # Facteur d'échelle : 1 m = 100 pixels


# CINÉMATIQUE DU ROBOT (roues Mécanum / omnidirectionnel 4 roues)


# Convention des roues :
#   FL = Front-Left  (avant-gauche)
#   FR = Front-Right (avant-droite)
#   BR = Back-Right  (arrière-droite)
#   BL = Back-Left   (arrière-gauche)
#
# Convention des vitesses corps (repère robot) :
#   vx    : vitesse longitudinale (avant positif)
#   vy    : vitesse latérale (gauche positif)
#   omega : vitesse de rotation (sens trigonométrique positif)
#
# Convention d'angle global (repère monde) :
#   theta : angle du robot, sens trigonométrique, 0 = axe X positif


def rpm_to_rad_s(rpm: float) -> float:
    """Convertit des RPM en rad/s."""
    return rpm * (2.0 * math.pi / 60.0)


def rpm_to_body_velocities(fl_rpm: float, fr_rpm: float,
                            br_rpm: float, bl_rpm: float):
    """
    Cinématique directe : RPM des 4 roues -> vitesses du corps (repère robot).

    Retourne :
        vx    (m/s) : vitesse longitudinale (avant = positif)
        vy    (m/s) : vitesse latérale (gauche = positif)
        omega (rad/s) : vitesse angulaire (trigonométrique = positif)
    """
    w_fl = rpm_to_rad_s(fl_rpm) * GEAR_RATIO
    w_fr = rpm_to_rad_s(fr_rpm) * GEAR_RATIO
    w_br = rpm_to_rad_s(br_rpm) * GEAR_RATIO
    w_bl = rpm_to_rad_s(bl_rpm) * GEAR_RATIO

    r = WHEEL_RADIUS
    l = LX + LY

    vx    = (r / 4.0) * ( w_fl + w_fr + w_br + w_bl)
    vy    = (r / 4.0) * (-w_fl + w_fr - w_br + w_bl)
    omega = (r / (4.0 * l)) * (-w_fl + w_fr + w_br - w_bl)

    return vx, vy, omega


def body_to_world_velocities(vx: float, vy: float,
                              omega: float, theta: float):
    """
    Projette les vitesses du repère robot dans le repère monde.

    Retourne :
        vx_w, vy_w (m/s) : vitesses dans le repère monde
        omega      (rad/s): inchangé (rotation = scalaire)
    """
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    vx_w = vx * cos_t - vy * sin_t
    vy_w = vx * sin_t + vy * cos_t
    return vx_w, vy_w, omega


def integrate_odometry(x: float, y: float, theta: float,
                       vx: float, vy: float, omega: float,
                       dt: float):
    """
    Intègre les vitesses monde pour mettre à jour la pose.

    Retourne :
        x_new, y_new (m), theta_new (rad) dans [-pi, pi]
    """
    x_new     = x     + vx    * dt
    y_new     = y     + vy    * dt
    theta_new = theta + omega * dt
    theta_new = angle_wrap(theta_new)
    return x_new, y_new, theta_new


def angle_wrap(angle: float) -> float:
    """Ramène un angle dans [-pi, pi]."""
    while angle >  math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle