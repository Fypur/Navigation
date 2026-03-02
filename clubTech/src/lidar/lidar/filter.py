import numpy as np
from lidar.dbscan import *

class LidarFilter:
    def __init__(self, x_max = 3.0, y_max = 2.0, x_min = 0.0, y_min = 0.0):
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max  # Longueur de la table en mètres
        self.y_max = y_max  # Largeur de la table en mètres
    
    def filter(self, scan_msg, robot_pos):
        n = len(scan_msg.ranges)
        pos_x,pos_y,pos_z,qx,qy,qz,qw = robot_pos
        alpha = np.pi + np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)) #alpha c'est l'angle que fait le robot avec l'axe des x

        thetas = []
        distances = []

        theta = scan_msg.angle_min

        for r in scan_msg.ranges:
            thetas.append(theta)
            distances.append(r)

            theta += scan_msg.angle_increment

        filtered_ranges = scan_msg.ranges[:]
        filtered_coordinates = []

        for i in range(n):
            theta, d = thetas[i], distances[i]
            x_obs = pos_x + d*np.cos(alpha+theta) # On note la position des obstacles
            y_obs = pos_y + d*np.sin(alpha+theta)

            if self.x_min <= x_obs <= self.x_max and self.y_min <= y_obs <= self.y_max:
                filtered_ranges[i] = d
                filtered_coordinates.append((x_obs, y_obs))
            else:
                filtered_ranges[i] = np.inf
                
        return (filtered_ranges, filtered_coordinates)
    
    def opp_pos(self, obs_coordinates, D = 0.141):
        """
            Reçois en entrée la liste filtrés des points représentants les obstacles
            et retourne la coordonnée (x,y) du centre du robot adverse.
            Ici D représente la diagonale de la balise embarquée sur le robot qui est
            au maximum un carré de 100 mm de côté.
        """

        pts_cluster = largest_cluster(obs_coordinates, D)
        sum_x,sum_y = 0,0
        for x, y in pts_cluster:
            sum_x += x
            sum_y += y

        n = len(pts_cluster)
        if n == 0:
            return (np.inf,np.inf) # Si aucun cluster n'est détecté, alors on considère que l'ennemi est situé à une distance infinie
        G_mass = (sum_x / n, sum_y / n)

        opp_pos = G_mass

        return(opp_pos)








