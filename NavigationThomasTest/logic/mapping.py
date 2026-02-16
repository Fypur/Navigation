import sys
import os
import json
import time
import numpy as np
import matplotlib
matplotlib.use('Agg') # Mode sans fenêtre graphique (pour serveur)
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

# Import config
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import paho.mqtt.client as mqtt
from config import constants

# --- VARIABLES GLOBALES ---
current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
global_map_points = [] # Liste de tous les points obstacles trouvés (X, Y global)

# --- FONCTIONS MATHS ---
def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y

def transform_local_to_global(local_points, robot_pose):
    """
    Transforme les points du repère robot au repère monde.
    local_points: liste de (x, y)
    robot_pose: {x, y, theta}
    """
    global_pts = []
    cos_theta = np.cos(robot_pose['theta'])
    sin_theta = np.sin(robot_pose['theta'])
    
    for lx, ly in local_points:
        # Rotation + Translation
        gx = (lx * cos_theta - ly * sin_theta) + robot_pose['x']
        gy = (lx * sin_theta + ly * cos_theta) + robot_pose['y']
        global_pts.append([gx, gy])
    
    return np.array(global_pts)

# --- MQTT CALLBACKS ---
def on_connect(client, userdata, flags, rc):
    print("Mapping Service connecté.")
    client.subscribe(constants.TOPIC_ODOM)
    client.subscribe(constants.TOPIC_LIDAR)

def on_message(client, userdata, msg):
    global current_pose, global_map_points
    
    try:
        # 1. Mise à jour de la position robot
        if msg.topic == constants.TOPIC_ODOM:
            data = json.loads(msg.payload.decode())
            current_pose = data 
            # Note: data contient x, y, theta

        # 2. Mise à jour de la carte (Quand on reçoit un scan LiDAR)
        elif msg.topic == constants.TOPIC_LIDAR:
            lidar_data = json.loads(msg.payload.decode())
            
            # Conversion Polaire (LiDAR) -> Cartésien Local (Robot)
            local_points = []
            for pt in lidar_data:
                # Attention : fichiers précédents utilisaient -angle ou +angle + pi/2
                # On reste standard : X devant, Y gauche.
                # LiDAR souvent : 0° devant.
                angle_rad = np.radians(pt['a'])
                dist_m = pt['d'] / 1000.0 # Conversion mm -> m
                
                if dist_m < 0.1 or dist_m > 4.0: continue # Filtre distance min/max

                lx, ly = pol2cart(dist_m, angle_rad)
                local_points.append([lx, ly])

            if not local_points: return

            # Conversion Cartésien Local -> Cartésien Global
            new_global_points = transform_local_to_global(local_points, current_pose)
            
            # Ajout à la mémoire (Simplification : on garde tout pour l'instant)
            # carte "nuage de points" simple.
            
            # Pour éviter d'exploser la mémoire, on garde seulement les 2000 derniers points
            global_map_points.extend(new_global_points.tolist())
            if len(global_map_points) > 2000:
                global_map_points = global_map_points[-2000:]

    except Exception as e:
        print(f"Erreur Mapping: {e}")

# --- BOUCLE DE GÉNÉRATION D'IMAGE ---
def save_map_image():
    global global_map_points
    
    if len(global_map_points) < 10: return
    
    points_np = np.array(global_map_points)
    
    plt.figure(figsize=(10, 10))
    plt.axis('equal') # Garder les proportions
    
    # 1. Dessiner les points bruts (Gris)
    plt.scatter(points_np[:, 0], points_np[:, 1], s=2, c='gray', alpha=0.5)
    
    # 2. Dessiner le robot (Rouge)
    plt.plot(current_pose['x'], current_pose['y'], 'ro', markersize=10)
    # Flèche de direction
    plt.arrow(current_pose['x'], current_pose['y'], 
              0.2 * np.cos(current_pose['theta']), 
              0.2 * np.sin(current_pose['theta']), 
              head_width=0.05, head_length=0.1, fc='r', ec='r')

    # 3. Clustering & Convex Hull
    # On applique DBSCAN sur les points globaux pour trouver les "objets"
    if len(points_np) > 10:
        try:
            db = DBSCAN(eps=0.3, min_samples=5).fit(points_np) # eps = 30cm
            labels = db.labels_
            unique_labels = set(labels)
            
            for label in unique_labels:
                if label == -1: continue # Bruit
                
                class_member_mask = (labels == label)
                cluster_points = points_np[class_member_mask]
                
                if len(cluster_points) >= 3:
                    hull = ConvexHull(cluster_points)
                    # Dessiner le contour
                    for simplex in hull.simplices:
                        plt.plot(cluster_points[simplex, 0], cluster_points[simplex, 1], 'b-')
        except Exception as e:
            pass # Si ça plante (points alignés), on continue sans dessiner les boites

    plt.grid(True)
    plt.title(f"Carte Globale - Robot à ({round(current_pose['x'],2)}, {round(current_pose['y'],2)})")
    
    # Sauvegarde dans le dossier static du site web pour affichage
    output_path = os.path.join(os.path.dirname(__file__), '..', 'web', 'static', 'map.png')
    plt.savefig(output_path)
    plt.close()
    print("Carte mise à jour.")

if __name__ == '__main__':
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(constants.MQTT_BROKER, constants.MQTT_PORT, 60)
    client.loop_start()

    # Boucle lente pour générer l'image
    try:
        while True:
            time.sleep(2) # Mise à jour de l'image toutes les 2 secondes
            save_map_image()
    except KeyboardInterrupt:
        client.loop_stop()