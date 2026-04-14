"""
Noeud Localization - Odométrie et Fusion de capteurs (Encodeurs + Lidar)

Ce noeud calcule et met à jour en temps réel l'estimation de la position (x, y) 
et de l'orientation (theta) du robot par rapport à son point de départ.

Il utilise une approche de fusion à deux niveaux :
  1. Odométrie cinématique (Haute fréquence) : Intègre les mouvements mesurés par 
     les encodeurs des roues. C'est très réactif et fluide, mais sujet à la dérive (glissement).
  2. Odométrie Lidar (Basse fréquence) : Compare les nuages de points
     successifs (algo ICP) sur l'environnement fixe pour calculer un offset et 
     corriger la dérive des roues.

S'abonne à :
    - /robot/encoders (msgs/RPMs) : tics/vitesses des 4 moteurs.
    - /robot/lidar (msgs/Lidar) : nuage de points 360° complet de l'environnement.

Publie sur :
    - /robot/pos (geometry_msgs/Pose2D) : la position finale corrigée, destinée
      aux noeuds de navigation (comme le noeud automatic).
"""

import rclpy
from rclpy.node import Node
from msgs.msg import Lidar, RPMs
from geometry_msgs.msg import Pose2D
import math

# -- Paramètres physiques du robot (à ajuster) --
WHEEL_RADIUS = 0.04  # Rayon de la roue en mètres (ex: 4cm)
LX = 0.15           # Distance entre le centre et l'axe des roues avant/arrière (m)
LY = 0.15           # Distance entre le centre et l'axe des roues gauche/droite (m)
GEAR_RATIO = 1.0     # Si RPM sont ceux du moteur et non de la roue

class LocalizationNode(Node):
    
    def __init__(self):
        super().__init__('localization')
        
        # -- Etat interne de la position -- 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # -- Pour la comparaison avec la position -- 
        self.previous_lidar_scan = None
        
        # -- Abonnements aux topics --
        
        # Abonnement aux encodeurs pour la premmière estimation de la position
        self.create_subscription (RPMs, '/robot/encoders', self.encoders_callback,50)
        
        # Abonnement au Lidar pour la correction (Basse fréquence)
        self.create_subscription (Lidar, '/robot/lidar', self.lidar_callback, 10)
        
        # -- Publication (Output) --
        # Remplace le faux positionnement absolu pour le noeud Automatic
        self.pub_pos = self.create_publisher(Pose2D, '/robot/pos', 10)
        
        self.get_logger().info("Noeud Localization (Encodeurs et Lidar) démarré")
        
    # -- Callback pour les encodeurs --
    def encoders_callback(self, msg: RPMs):
        """
        Calcul cinématique pur : mise à jour rapide mais qui dérive.
        """
        
        # Calculer dx, dy, dtheta depuis les vitesses/ticks des 4 roues 
        # (Cinématique directe du robot)
        
        dx, dy, dtheta = self._compute_kinematics(msg)
        
        # Mettre à jour la position interne
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dtheta
        self.theta = self._angle_wrap(self.theta)
        
        # Publier la position estimee fluide
        self._publish_pose()
        
    # -- Callback lent : Correction Lidar --
    def lidar_callback(self, msg: Lidar):
        """
        Recalage de la position via l'environnement pour annuler la dérive
        des encodeurs.
        """
        current_scan = msg
        
        if self.previous_lidar_scan is not None:
            # 1. Executer un algorithme de type ICP (Iterative Closest Point)
            # Compare current_scan et previous_lidar_scan
            correction_x, correction_y, correction_theta = self._run_icp(self.previous_lidar_scan, current_scan)
            
            #2. Appliquer la correction à la position interne
            self.x += correction_x
            self.y += correction_y
            self.theta += correction_theta
            self.theta = self._angle_wrap(self.theta)
            
            # On ne publie pas ici et on laisse la position corrigée 
            # être publiée par le callback des encodeurs
            
        self.previous_lidar_scan = current_scan
        
    
    # -- Méthodes utilitaires --
    
    def _publish_pose(self):
        msg = Pose2D()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.pub_pos.publish(msg)
    
    def _compute_kinematics(self, msg):
    # 1. Conversion RPM -> Vitesse angulaire de la roue (rad/s)
    # Formule : RPM * 2pi / 60
        coef = (2 * math.pi) / 60.0
        w_fl = msg.front_left_rpm * coef
        w_fr = msg.front_right_rpm * coef
        w_rl = msg.back_left_rpm * coef
        w_rr = msg.back_right_rpm * coef

        # 2. Vitesse linéaire du robot (repère local robot)
        # Modèle cinématique inverse pour châssis 4 roues
        vx = (WHEEL_RADIUS / 4.0) * ( w_fl + w_fr + w_rl + w_rr)
        vy = (WHEEL_RADIUS / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        w  = (WHEEL_RADIUS / (4.0 * (LX + LY))) * (-w_fl + w_fr - w_rl + w_rr)

        # 3. Calcul du déplacement (vitesse * temps entre deux messages)
        # On suppose ici une fréquence de 50Hz (dt = 0.02s)
        dt = 0.02 
        return vx * dt, vy * dt, w * dt # A REMPLIR (calcul du déplacement à partir des vitesses)
    
    def _run_icp(self, scan_prev, scan_curr):
        """
        Calcule la correction entre deux scans successifs.
        """
        # 1. Transformer scan_prev et scan_curr en listes de points [(x1,y1), (x2,y2)...]
        points_prev = self._polar_to_cartesian(scan_prev)
        points_curr = self._polar_to_cartesian(scan_curr)

        # 2. Ici, on devrait normalement utiliser une bibliothèque
        # Pour un test initial, on peut renvoyer 0,0,0 ou une estimation basée sur 
        # la moyenne des centroïdes.
        
        correction_x, correction_y, correction_theta = 0.0, 0.0, 0.0
        
        return correction_x, correction_y, correction_theta # A MODIFIER (calcul de la correction via ICP ou une méthode simple)


    def _polar_to_cartesian(self, scan):
        points = []
        for angle, dist in zip(scan.angles, scan.distances):
            if math.isfinite(dist) and dist > 0.05:
                px = dist * math.cos(angle)
                py = dist * math.sin(angle)
                points.append((px, py))
        return points

    
    def _angle_wrap(self, angle):
        # Ramène un angle à [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
