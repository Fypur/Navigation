import rclpy
from rclpy.node import Node
from msgs.msg import Lidar, RPMs
from geometry_msgs.msg import Pose2D
import math
import numpy as np
import time
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

WHEEL_RADIUS = 0.04
LX = 0.15
LY = 0.15
GEAR_RATIO = 1.0

def best_rigid_transform(A, B):
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A

    return R, t


# -----------------------------
# Nearest neighbor
# -----------------------------
def nearest_neighbor(src, dst):
    indices = []
    for p in src:
        dists = np.linalg.norm(dst - p, axis=1)
        indices.append(np.argmin(dists))
    return dst[indices]




class LocalizationNode(Node):
    
    def __init__(self):
        super().__init__('localization')
        
        # -- Etat interne de la position -- 
        self.x = 0.0
        self.y = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.theta = 0.0
        self.prev_theta = 0.0
        self.last_time = time.time()
        
        # -- Abonnements aux topics --
        
        # Abonnement aux encodeurs pour la premmière estimation de la position
        self.create_subscription (RPMs, '/robot/encoders', self.encoders_callback, 1)
        
        # Abonnement au Lidar pour la correction (Basse fréquence)
        #self.create_subscription (Lidar, '/robot/lidar', self.lidar_callback, 1)
        
        # -- Publication --
        self.pub_pos = self.create_publisher(Pose2D, '/robot/pos', 10)
        
        self.get_logger().info("Noeud Localization (Encodeurs et Lidar) démarré")
        

    # -- Callback pour les encodeurs --
    def encoders_callback(self, msg: RPMs):
        """
        Calcul cinématique pur : mise à jour rapide mais qui dérive.
        """
        # Conversion RPM -> Vitesse angulaire de la roue (rad/s)
        coef = (2 * math.pi) / 60.0
        w_fl = msg.front_left_rpm * coef
        w_fr = msg.front_right_rpm * coef
        w_rl = msg.back_left_rpm * coef
        w_rr = msg.back_right_rpm * coef
        
        # Modele cinématique inverse pour châssis 4 roues (Mécanum)
        vx = (WHEEL_RADIUS / 4.0) * ( w_fl + w_fr + w_rl + w_rr)
        vy = (WHEEL_RADIUS / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        w  = (WHEEL_RADIUS / (4.0 * (LX + LY))) * (-w_fl + w_fr - w_rl + w_rr)
        
        # Calcul temps réel
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 1.0 or dt <= 0.0:
            dt = 0.1
        
        # Déplacement dans le repère local -> projeté dans le repère global
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += w * dt
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
            # On projette le scan reçu dans le repère global en utilisant la position dérivée
            #pts_curr_global = self._get_global_points(msg)

            # Initialisation
            if not hasattr(self, 'prev_global_points'):
                #self.prev_global_points = pts_curr_global
                #self.prev_x, self.prev_y, self.prev_theta = self.x, self.y, self.theta
                self.prev_x, self.prev_y, self.prev_theta = self.x, self.y, self.theta
                self.prev_global_points = self._get_global_points(msg, self.prev_x, self.prev_y, self.prev_theta)
                
                #print(self.prev_global_points)
                #print(self.prev_x, self.prev_y, self.prev_theta)
                return

            pts_curr_global = self._get_global_points(msg, self.prev_x, self.prev_y, self.prev_theta)
            
            
            
            
            # Calcul de la transformation ICP entre le scan actuel et le scan précédent
            R, translation, dtheta = self._run_icp_papa(self.prev_global_points, pts_curr_global)
            

            if R is not None:
                self.x,self.y,self.theta = self.update_robot_pose(self.prev_x, self.prev_y, self.prev_theta, dtheta, translation)
                # Appliquer la transformation mathématique à la position du robot
                #robot_pos = np.array([self.prev_x, self.prev_y])
                #new_pos = R @ robot_pos + translation
                
                #self.x = new_pos[0]
                #self.y = new_pos[1]
                #self.theta = self._angle_wrap(self.prev_theta + dtheta)
                #print('translation : {translation}, R : {R}'.format(translation=translation, R=R))

                # Le nouveau scan "corrigé" devient la référence pour le prochain coup
                self.prev_global_points = pts_curr_global #self._get_global_points(msg)
                self.prev_x, self.prev_y, self.prev_theta = self.x, self.y, self.theta
                
            else:
                # Si l'ICP échoue (peu de murs), on met à jour la référence avec l'odométrie
                self.prev_global_points = pts_curr_global
                self.prev_x, self.prev_y, self.prev_theta = self.x, self.y, self.theta

            #print(self.prev_global_points)
            #print(self.prev_x, self.prev_y, self.prev_theta)
            
            self._publish_pose()

        except Exception as e:
            self.get_logger().error(f"Erreur ICP Lidar : {e}")
        
    
    # -- Méthodes utilitaires --
    
    def _get_global_points(self, scan, ref_x, ref_y, ref_theta):
        """Convertit un scan Lidar en nuage de points cartésiens dans le repère Global."""
        points = []
        cos_t = math.cos(ref_theta)
        sin_t = math.sin(ref_theta)
        
        for angle, dist in zip(scan.angles, scan.distances):
            # On accepte les obstacles jusqu'à 5.95m
            # Les points à 6.0m sont ignorés car c'est le "vide" renvoyé par la simulation
            if math.isfinite(dist) and 0.05 < dist < 5.95: 
                lx = dist * math.cos(angle)
                ly = dist * math.sin(angle)
                gx = ref_x + lx * cos_t - ly * sin_t
                gy = ref_y + lx * sin_t + ly * cos_t
                points.append((gx, gy))
                
        return np.array(points)
    
    
    '''def _run_icp_global(self, pts_prev, pts_curr):
        """
        Trouve la matrice de Rotation (R) et de Translation (T) qui aligne pts_curr sur pts_prev
        via l'algorithme Iterative Closest Point (ICP).
        """
        if len(pts_prev) < 20 or len(pts_curr) < 20:
            return None, None, None

        tree = KDTree(pts_prev)
        src_pts = np.copy(pts_curr)
        
        total_R = np.eye(2)
        total_t = np.zeros(2)
        
        max_iterations = 20
        
        for _ in range(max_iterations):
            # Trouver les points les plus proches
            distances, indices = tree.query(src_pts)
            
            # Filtrer les aberrations (points à plus de 30cm)
            #valid = distances < 0.3
            matched_src = src_pts#[valid]
            matched_prev = pts_prev[indices]#[valid]]
            
            if len(matched_src) < 20:
                break
                
            # Calcul des centres de gravité
            c_src = np.mean(matched_src, axis=0)
            c_prev = np.mean(matched_prev, axis=0)
            
            # Centrer les points
            p_src_centered = matched_src - c_src
            p_prev_centered = matched_prev - c_prev
            
            # Calcul de l'alignement via SVD
            H = p_src_centered.T @ p_prev_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Gérer les réflexions miroir éventuelles
            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T
                
            t = c_prev - R @ c_src
            
            # Appliquer la transformation locale au nuage pour la prochaine itération
            src_pts = (R @ src_pts.T).T + t
            
            # Cumuler la transformation globale (C'est ce qu'on renverra au robot)
            total_R = R @ total_R
            total_t = R @ total_t + t
            
            # Condition d'arrêt : Si l'écart moyen est minuscule, on a convergé
            if np.mean(distances) < 0.005 : #[valid]) < 0.005: 
                break
        
        dtheta = math.atan2(total_R[1, 0], total_R[0, 0])
        return total_R, total_t, dtheta'''
    
    
    def _run_icp_papa(self, pts_prev, pts_curr, max_iter=50):
        src = pts_curr.copy()
        target = pts_prev

        R_total = np.eye(2)
        t_total = np.zeros(2)

        for _ in range(max_iter):
            matched = nearest_neighbor(src, target)
            R, t = best_rigid_transform(src, matched)

            src = (R @ src.T).T + t

            R_total = R @ R_total
            t_total = R @ t_total + t

        theta = np.arctan2(R_total[1, 0], R_total[0, 0])
        
        return R_total, t_total, theta

    def icp_to_robot_motion(self, theta_icp, t_icp):
        # matrice rotation ICP
        R_icp = np.array([
            [np.cos(theta_icp), -np.sin(theta_icp)],
            [np.sin(theta_icp),  np.cos(theta_icp)]
        ])

        # inversion
        R_robot = R_icp.T
        t_robot = -R_robot @ t_icp

        theta_robot = np.arctan2(R_robot[1, 0], R_robot[0, 0])

        return theta_robot, t_robot


    def update_robot_pose(self, x, y, theta, theta_icp, t_icp):
        theta_robot, t_robot = self.icp_to_robot_motion(theta_icp, t_icp)

        # rotation du déplacement dans le repère monde
        R_world = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

        t_world = R_world @ t_robot

        x_new = x + t_world[0]
        y_new = y + t_world[1]
        theta_new = theta + theta_robot
        
        '''x_new = math.cos(theta_icp) * x - math.sin(theta_icp) * y + t_icp[0]
        y_new = math.sin(theta_icp) * x + math.cos(theta_icp) * y + t_icp[1]
        
        # Mise à jour de l'angle (addition directe)
        theta_new = self._angle_wrap(theta + theta_icp)'''

        return x_new, y_new, theta_new
    
    def _publish_pose(self):
        msg = Pose2D()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.theta = float(self.theta)
        self.pub_pos.publish(msg)
    
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
