import rclpy
from rclpy.node import Node
from msgs.msg import Lidar, Encoders
from geometry_msgs.msg import Pose2D
import math

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
        self.create_subscription (Encoders, '/robot/encodders', self.encoders_callback,50)
        
        # Abonnement au Lidar pour la correction (Basse fréquence)
        self.create_subscription (Lidar, '/robot/lidar', self.lidar_callback, 10)
        
        # -- Publication (Output) --
        # Remplace le faux positionnement absolu pour le noeud Automatic
        self.pub_pos = self.create_publisher(Pose2D, '/robot/pos', 10)
        
        self.get_logger().info("Noeud Localization (Encodeurs et Lidar) démarré")
        
    # -- Callback pour les encodeurs --
    def encoders_callback(self, msg: Encoders):
        """
        Calcul cinématique pur : mise à jour rapide mais qui dérive.
        """
        
        # Calculer dx, dy, dtheta depuis les vitesses/ticks des 4 roues 
        # (Cinématique directe du robot)
        
        dx, dy, dtheta = self._compute_kinematics(msg)
        
        # Mettre à jour la position interne
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
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
        # Implémentation mathématique des roues vers (dx, dy, dtheta)
        return 0.0, 0.0, 0.0
    
    def _run_icp(self, scan1: Lidar, scan2: Lidar):
        # Implémentation d'un algorithme de recalage de nuages de points
        # pour trouver la transformation (correction_x, correction_y, correction_theta)
        return 0.0, 0.0, 0.0
    
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
