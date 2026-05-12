import math

# -- Console --
DEFAULT_RPM = 150.0  # RPM par défaut envoyé aux roues

# -- Automatic --
K_ATT                 = 8.0            # Gain du champ attractif #1.5
K_REP                 = 0.8            # Gain du champ répulsif
APF_D0                = 2.0            # Rayon d'influence des obstacles (m) — doit correspondre à LIDAR_OBSTACLE_RANGE
APF_MIN_DIST          = 0.05           # Distance plancher pour éviter la division par zéro (m)
GOAL_RADIUS           = 0.2            # Distance en dessous de laquelle la cible est considérée atteinte (m)
DEFAULT_GOAL_X        = 1.0            # Position cible par défaut en x (m)
DEFAULT_GOAL_Y        = 0.0            # Position cible par défaut en y (m)
APF_MAX_LINEAR_FORCE  = 3.0            # Force maximale appliquée par l'APF convertie en vitesse
APF_MAX_ANGULAR_FORCE = 2.0            # Force angulaire max
AUTO_LOOP_HZ          = 10.0           # Fréquence de la boucle de controle automatique (Hz)
APF_HEADING_GAIN      = 0.2            # Gain pour l'alignement angulaire APF
APF_HEADING_TOLERANCE = math.pi / 4.0  # Angle à partir duquel le robot réduit sa vitesse linéaire pour privilégier la rotation et éviter les zigzags

# -- Localization (ICP et Cartographie) --
LOC_DIST_THRESHOLD        = 0.3               # Distance mini pour maj de la carte (m)
LOC_ANGLE_THRESHOLD       = math.radians(15)  # Angle mini pour maj de la carte (rad)
ICP_MAX_CORRESPOND_DIST   = 1.5               # Distance max pour qu'une correspondance soit valide (m)
ICP_MAX_JUMP_DIST         = 0.3              # Tolérance de saut de distance brutale ICP (m)
ICP_MAX_JUMP_ANGLE        = 0.40              # Tolérance de saut d'angle brutale ICP (rad)
ICP_VOXEL_SIZE            = 0.05              # Taille du voxel pour le filtrage du nuage de points (m)
ICP_FITNESS_THRESHOLD     = 0.7               # Score min pour accepter la correction (0.0 à 1.0)
ICP_INLIER_RMSE_THRESHOLD = 0.15              # Erreur de distance max acceptée (m)
ICP_MAP_UPDATE_FITNESS    = 0.85              # Score min de fitness pour autoriser l'ajout à la carte
ICP_MIN_POINTS            = 30                # Nombre minimum de points requis pour lancer l'algo ICP 


# -- Paramètres généraux du robot --
WHEEL_RADIUS = 0.0325  # Rayon des roues en mètres (3.25 cm)
ROBOT_RADIUS = 0.15  # Rayon approximatif du robot pour l'APF (m)
LX           = 0.15  # Distance centre -> axe avant/arrière (m)
LY           = 0.15  # Distance centre -> axe gauche/droite (m)
GEAR_RATIO   = 1.0   # Rapport de réduction (1.0 = RPM moteur == RPM roue)


# -- Paramètres des actionneurs --
MAX_RPM = 160.0  # RPM maximum envoyé aux roues (même échelle que la console)
MIN_RPM = 100.0  # RPM minimum envoyé aux roues 


# -- Paramètres du Lidar --
LIDAR_MAX_RANGE_M    = 6.0   # Portée maximale du lidar (m)
LIDAR_N_RAYS         = 360   # Nombre de rayons / Simulation
LIDAR_MIN_DIST       = 0.05  # Distance plancher pour ignorer les points trop proches (m)
LIDAR_OBSTACLE_RANGE = 2.0   # Distance en dessous de laquelle un point est un "obstacle" (m)

# -- Paramètres des encodeurs --
ENCODER_NOISE_THRESHOLD = 0.5



# CINÉMATIQUE DU ROBOT


# Convention des roues :
#   FL = Front-Left  (avant-gauche)
#   FR = Front-Right (avant-droite)
#   BR = Back-Right  (arrière-droite)
#   BL = Back-Left   (arrière-gauche)



def angle_wrap(angle: float) -> float:
    """Ramène un angle dans [-pi, pi]."""
    while angle >  math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle