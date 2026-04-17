import rclpy
from rclpy.node import Node
from msgs.msg import Lidar, RPMs
from geometry_msgs.msg import Pose2D
import math
import numpy as np
from scipy.spatial import KDTree

# -- Paramètres physiques du robot (à ajuster) --
WHEEL_RADIUS = 0.04  # Rayon de la roue en mètres (ex: 4cm)
LX = 0.15            # Distance entre le centre et l'axe des roues avant/arrière (m)
LY = 0.15            # Distance entre le centre et l'axe des roues gauche/droite (m)
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
        
        # -- Publication --
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
        
        try:
            # L'ICP a besoin de DEUX scans pour comparer.
            if not hasattr(self, 'prev_scan'):
                self.prev_scan = msg
                return # On arrête là pour cette fois
            
            # On lance l'ICP
            dx, dy, dtheta = self._run_icp(self.prev_scan, msg)

            # Mise à jour de la position
            self.x += dx
            self.y += dy
            self.theta += dtheta

            # Création et publication du message
            pose_msg = Pose2D()
            pose_msg.x = float(self.x)
            pose_msg.y = float(self.y)
            pose_msg.theta = float(self.theta)
            
            self.pub_pos.publish(pose_msg)

            # On sauvegarde le scan actuel pour le prochain tour
            self.prev_scan = msg

        except Exception as e:
            # SI LE MOINDRE TRUC PLANTE, ÇA S'AFFICHERA ICI EN ROUGE
            self.get_logger().error(f"ERREUR FATALE : {e}")
        
    
    # -- Méthodes utilitaires --
    
    def _publish_pose(self):
        msg = Pose2D()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.pub_pos.publish(msg)
    
    def _compute_kinematics(self, msg):
        # Conversion RPM -> Vitesse angulaire de la roue (rad/s)
        # Formule : RPM * 2pi / 60
        coef = (2 * math.pi) / 60.0
        w_fl = msg.front_left_rpm * coef
        w_fr = msg.front_right_rpm * coef
        w_rl = msg.back_left_rpm * coef
        w_rr = msg.back_right_rpm * coef

        # Vitesse linéaire du robot (repère local robot)
        # Modèle cinématique inverse pour châssis 4 roues
        vx = (WHEEL_RADIUS / 4.0) * ( w_fl + w_fr + w_rl + w_rr)
        vy = (WHEEL_RADIUS / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        w  = (WHEEL_RADIUS / (4.0 * (LX + LY))) * (-w_fl + w_fr - w_rl + w_rr)

        # Calcul du déplacement (vitesse * temps entre deux messages)
        # On suppose ici une fréquence de 50Hz (dt = 0.02s)
        dt = 0.02 
        return vx * dt, vy * dt, w * dt # A REMPLIR (calcul du déplacement à partir des vitesses)
    
    def _run_icp(self, scan_prev, scan_curr):
        """
        Calcule la correction entre deux scans successifs.
        """
        # Transformer scan_prev et scan_curr en listes de points [(x1,y1), (x2,y2)...]
        pts_prev = np.array(self._polar_to_cartesian(scan_prev))
        pts_curr = np.array(self._polar_to_cartesian(scan_curr))

        # On utilise une bibliothèque spéciale
        if len(pts_prev) < 10 or len(pts_curr) < 10:
            return 0.0, 0.0, 0.0 # Pas assez de points pour une correction fiable   
        
        # Recherche des correspondances (Plus proches voisins)
        tree = KDTree(pts_prev)
        distances, indices = tree.query(pts_curr)
        
        # On garde les points assez proches pour éviter les erreurs
        valid = distances < 0.2
        matched_prev = pts_prev[indices[valid]]
        matched_curr = pts_curr[valid]
        
        # Calcul des centres des clusters
        c_prev = np.mean(matched_prev, axis=0)
        c_curr = np.mean(matched_curr, axis=0)
        
        # Calcul de la rotation
        p_shifted = matched_prev - c_prev
        q_shifted = matched_curr - c_curr
        H = p_shifted.T @ q_shifted
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        dx, dy= c_curr - R @ c_prev
        
        return dx, dy, dtheta


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
