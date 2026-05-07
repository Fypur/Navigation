import random

import rclpy
from rclpy.node import Node
from msgs.msg import Lidar, RPMs
from geometry_msgs.msg import Pose2D
import math
import numpy as np
import time
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import pymunk

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
        #self.prev_global_points = None
        self.last_time = time.time()
        self.walls = None
        
        plt.ion() # Active le mode interactif (non-bloquant)
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.fig.canvas.manager.set_window_title('Localisation Lidar Live')
        
        # -- Abonnements aux topics --
        
        # Abonnement aux encodeurs pour la premmière estimation de la position
        self.create_subscription (RPMs, '/robot/encoders', self.encoders_callback, 1)
        
        # Abonnement au Lidar pour la correction (Basse fréquence)
        self.create_subscription (Lidar, '/robot/lidar', self.lidar_callback, 1)
        
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
        
        if w_fl < 5 and w_fr < 5 and w_rl < 5 and w_rr < 5:
            # Si les roues tournent à moins de 5 RPM, on considère que le robot est à l'arrêt
            # pour éviter les dérives dues au bruit des encodeurs à basse vitesse.
            return
        
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
        
    def _extract_walls_from_scan(self, msg: Lidar, rx, ry, r_theta):
        """
        Reconstruit les murs en zippant les angles et les distances du message ROS.
        """
        MAX_SEGMENT_DIST = 15#0.5
        LIDAR_RANGE = 6.0
        MAX_GAP_DIST = 0.5  # Distance max entre deux points pour rester sur le même mur
        new_walls = [
            # Murs extérieurs
            ((-3.5, 3.5), (3.5, 3.5)),
            ((3.5, 3.5), (3.5, -3.5)),
            ((3.5, -3.5), (-3.5, -3.5)),
            ((-3.5, -3.5), (-3.5, 3.5)),
            
            # Obstacles intérieurs
            ((-3.5, 2.0), (-1.5, 0.0)),
            ((2.0, 2.5), (2.0, -1.5)),
            ((-2.0, -2.5), (0.5, -2.5))
        ]
        return new_walls
        start_point = None
        prev_point = None
        
        for angle_relatif, r in zip(msg.angles, msg.distances):
            # Si le rayon touche un obstacle valide
            if math.isfinite(r) and r < LIDAR_RANGE:
                angle_global = r_theta + angle_relatif
                gx = rx + r * math.cos(angle_global)
                gy = ry + r * math.sin(angle_global)
                curr_point = (gx, gy)
                
                if prev_point is None:
                    # 1. Aucun mur en cours, on démarre un nouveau mur
                    start_point = curr_point
                else:
                    # 2. Un mur est en cours, on vérifie la continuité
                    dist = math.hypot(gx - prev_point[0], gy - prev_point[1])
                    if dist >= MAX_GAP_DIST:
                        # Cassure (fin du mur ou nouvel obstacle) : on clôture le mur en cours
                        if start_point != prev_point:
                            new_walls.append((start_point, prev_point))
                        # Le point actuel devient le début du nouveau mur
                        start_point = curr_point
                
                # On avance
                prev_point = curr_point
                
            # Si le rayon est vide (ne touche rien)
            else:
                # 3. On clôture le mur en cours (s'il y en a un et qu'il fait plus d'un point)
                if start_point is not None and prev_point is not None and start_point != prev_point:
                    new_walls.append((start_point, prev_point))
                
                # On réinitialise pour attendre le prochain obstacle
                start_point = None
                prev_point = None
                
        # 4. En sortant de la boucle, on n'oublie pas de sauvegarder le tout dernier mur en cours
        if start_point is not None and prev_point is not None and start_point != prev_point:
            new_walls.append((start_point, prev_point))
            
        return new_walls
      
    def _plot_walls_live(self):
            """
            Met à jour la figure Matplotlib pour dessiner le robot et les murs extraits.
            """
            self.ax.clear() # Efface le dessin précédent
            LIDAR_RANGE = 6.0
            
            # Dessiner le robot (un point rouge)
            self.ax.plot(self.x, self.y, 'ro', markersize=8, label='Robot')
            
            # Dessiner une ligne indiquant l'orientation (cap) du robot
            dx = math.cos(self.theta) * 0.5
            dy = math.sin(self.theta) * 0.5
            self.ax.plot([self.x, self.x + dx], [self.y, self.y + dy], 'r-', linewidth=2)
            
            # Dessiner tous les segments de murs enregistrés (lignes bleues)
            for p1, p2 in self.walls:
                self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-', linewidth=2)
                
            # Paramètres d'affichage (garder des proportions égales pour ne pas déformer la carte)
            self.ax.set_aspect('equal')
            # Centrer la caméra sur le robot avec une marge de vision
            self.ax.set_xlim(self.x - LIDAR_RANGE, self.x + LIDAR_RANGE)
            self.ax.set_ylim(self.y - LIDAR_RANGE, self.y + LIDAR_RANGE)
            self.ax.set_title(f"Murs Locaux - {len(self.walls)} segments")
            self.ax.grid(True)
            
            # Mettre à jour l'interface graphique brièvement (permet de ne pas bloquer ROS)
            plt.pause(0.001)
    
    # -- Callback lent : Correction Lidar --
    def lidar_callback(self, msg: Lidar):
        """
        """
        LIDAR_SAMPLES = 360
        LIDAR_RANGE = 6.0
        
        def _simulate_lidar_scan(rx, ry, r_theta, walls, msg_angles):
            """
            Génère un scan Lidar théorique en tirant des rayons contre les murs connus.
            Remplace complètement le besoin d'un moteur physique comme Pymunk.
            """
            theo_distances = []
            max_range_m = 6.0
            
            for angle_relatif in msg_angles:
                global_angle = r_theta + angle_relatif
                
                # Point de départ (robot) et point d'arrivée (bout du rayon Lidar)
                x1, y1 = rx, ry
                x2 = x1 + math.cos(global_angle) * max_range_m
                y2 = y1 + math.sin(global_angle) * max_range_m
                
                closest_dist = max_range_m
                
                # --- Raycasting contre chaque segment de mur ---
                for p1, p2 in walls:
                    # Calcul de l'intersection de deux segments (Rayon Lidar VS Mur)
                    den = (p2[0] - p1[0]) * (y1 - y2) - (x1 - x2) * (p2[1] - p1[1])
                    
                    # Si le dénominateur est 0, le rayon et le mur sont parfaitement parallèles
                    if den == 0: 
                        continue
                    
                    # t = position sur le mur (0 = début du mur, 1 = fin du mur)
                    # u = position sur le rayon Lidar (0 = centre du robot, 1 = bout du laser à 6m)
                    t = ((p1[0] - x1) * (y1 - y2) - (p1[1] - y1) * (x1 - x2)) / den
                    u = ((p1[0] - x1) * (p2[1] - p1[1]) - (p1[1] - y1) * (p2[0] - p1[0])) / den
                    
                    # Si on touche le segment du mur (0 <= t <= 1) avec l'avant du rayon Lidar (0 <= u <= 1)
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        # La distance d'impact est simplement u * la longueur totale du laser
                        dist_m = u * max_range_m
                        
                        # On garde uniquement l'obstacle le plus proche pour cet angle
                        if dist_m < closest_dist:
                            closest_dist = dist_m
                            
                theo_distances.append(closest_dist)
                
            return theo_distances
        
        '''def get_lidar_scan(rx, ry, r_angle, walls):
            scan = []
            for i in range(LIDAR_SAMPLES):
                angle = r_angle + i * (2 * math.pi / LIDAR_SAMPLES)
                closest = LIDAR_RANGE
                x1, y1 = rx, ry
                x2, y2 = x1 + math.cos(angle) * LIDAR_RANGE, y1 + math.sin(angle) * LIDAR_RANGE
                for p1, p2 in walls:
                    den = (p2[0]-p1[0])*(y1-y2) - (x1-x2)*(p2[1]-p1[1])
                    if den == 0: continue
                    t = ((p1[0]-x1)*(y1-y2) - (p1[1]-y1)*(x1-x2)) / den
                    u = ((p1[0]-x1)*(p2[1]-p1[1]) - (p1[1]-y1)*(p2[0]-p1[0])) / den
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        dist = t * math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)# dist = u * LIDAR_RANGE
                        if dist < closest: closest = dist
                        
                scan.append((closest, angle - r_angle))
            return scan'''
        

        def compute_fitness(tx, ty, ta, real_scan, walls):
            theo_scan = _simulate_lidar_scan(tx, ty, ta, walls, real_scan.angles)
            score = 0.0

            for r,t in zip(real_scan.distances, theo_scan):
                #theo_dist = t[0]
                if math.isfinite(r):
                    score += abs(r - t)
            return score
        
        if self.walls is None:
            self.walls = self._extract_walls_from_scan(msg, self.x, self.y, self.theta)
            print(self.walls)
            return
        
        x = self.x
        y = self.y
        theta = self.theta
        best_pos = (x, y, theta)
        #real_scan = []
        #for i in range(len(msg.distances)):
            #real_scan.append((msg.distances[i], msg.angles[i]))
            
        best_score = compute_fitness(x, y, theta, msg, self.walls)
        print("before,best scrore = {:.2f}, x = {:.2f}, y = {:.2f}, theta = {:.1f}°".format(best_score, self.x, self.y, math.degrees(self.theta)))

        
        for _ in range(30): # Augmenté à 30 pour plus de stabilité
            cand_x = x + random.uniform(-0.15, 0.15)
            cand_y = y + random.uniform(-0.15, 0.15)
            cand_a = self._angle_wrap(theta + random.uniform(-0.1, 0.1))
            score = compute_fitness(cand_x, cand_y, cand_a, msg, self.walls)
            if score < best_score:
                best_score, best_pos = score, (cand_x, cand_y, cand_a)
                print(best_pos, best_score)
                
        if abs(best_pos[0] - self.x) > 0.05 or abs(best_pos[1] - self.y) > 0.05 or abs(self._angle_wrap(best_pos[2] - self.theta)) > math.radians(5):        
            self.x, self.y, self.theta = best_pos
            
        print("after, x = {:.2f}, y = {:.2f}, theta = {:.1f}°".format(self.x, self.y, math.degrees(self.theta)))
        self._publish_pose()
        
        self.walls = self._extract_walls_from_scan(msg, self.x, self.y, self.theta)
        self._plot_walls_live()
        
        
    
            
        
        # -- Méthodes utilitaires --
        
        def _get_global_points(self, scan, ref_x, ref_y, ref_theta):
            """Convertit un scan Lidar en nuage de points cartésiens dans le repère Global."""
            points = []
            cos_t = math.cos(ref_theta)
            sin_t = math.sin(ref_theta)
            
            for angle, dist in zip(scan.angles, scan.distances):
                # On accepte les obstacles jusqu'à 5.95m
                # Les points à 6.0m sont ignorés car c'est le "vide" renvoyé par la simulation
                if math.isfinite(dist): #and 0.05 < dist < 5.95: 
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
        #theta_robot, t_robot = self.icp_to_robot_motion(theta_icp, t_icp)

        # rotation du déplacement dans le repère monde
        """R_world = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

        t_world = R_world @ t_robot

        x_new = x + t_world[0]
        y_new = y + t_world[1]
        theta_new = theta + theta_robot"""
        
        '''x_new = math.cos(theta_icp) * x - math.sin(theta_icp) * y + t_icp[0]
        y_new = math.sin(theta_icp) * x + math.cos(theta_icp) * y + t_icp[1]
        
        # Mise à jour de l'angle (addition directe)
        theta_new = self._angle_wrap(theta + theta_icp)'''
        
        """x_new = x + t_robot[0]
        y_new = y + t_robot[1]
        theta_new = self._angle_wrap(theta + theta_robot)"""
        x_new = x + t_icp[0]
        y_new = y + t_icp[1]
        theta_new = self._angle_wrap(theta + theta_icp)

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
