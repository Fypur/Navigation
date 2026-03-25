import socket
import time
import numpy as np
from sklearn.cluster import DBSCAN, KMeans
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import warnings

# Supprime un avertissement peu pertinent lié au clustering
warnings.filterwarnings("ignore", category=UserWarning, message="Number of distinct clusters")

# Paramètres de configuration
HOST = 'localhost'
PORT = 8080
MAX_POINTS = 100           # Nombre max de points par obstacle avant de sous-clusteriser
EPS = 40                   # Distance max entre deux points pour qu’ils soient considérés comme voisins (DBSCAN)
MIN_SAMPLES = 10           # Nombre minimum de points pour qu’un cluster soit considéré valide

# Classe représentant un obstacle détecté
class Obstacle:
    def __init__(self, label, envelope, points):
        self.label = label                      # Nom/étiquette de l'obstacle
        self.envelope = envelope                # Enveloppe (contour) de l'obstacle
        self.points = points                    # Points LIDAR constituant l'obstacle
        self.centroid = np.mean(points, axis=0) if len(points) > 0 else np.array([0.0, 0.0])  # Centre de masse

    def __repr__(self):
        return f"{self.label} | Points: {len(self.points)} | Centroid: {self.centroid}"

# Fonction pour récupérer les données du LIDAR pendant une certaine durée
def get_data(duration=1.0):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind((HOST, PORT))
        print("Attente des données LIDAR...")
        measure = []
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                data, _ = server_socket.recvfrom(1024)
                str_data = data.decode("utf-8")[3:].split(" ")
                angle = float(str_data[1])
                distance = float(str_data[3])
                measure.append((-angle, distance))  # Stocke l’angle négatif et la distance
            except:
                continue  # Ignore les erreurs de décodage
        return measure

# Renvoie un rectangle englobant tous les points (bounding box)
def bounding_box(points):
    xmin, ymin = points.min(axis=0)
    xmax, ymax = points.max(axis=0)
    return np.array([[xmin, ymin], [xmin, ymax], [xmax, ymax], [xmax, ymin]])

# Approximation linéaire entre les deux points les plus éloignés
def get_wall_approx(points):
    dist_mat = np.linalg.norm(points[:, None] - points[None, :], axis=2)
    i, j = np.unravel_index(np.argmax(dist_mat), dist_mat.shape)
    return points[[i, j]]

# Tente de calculer l'enveloppe convexe des points ; si échec, alternative
def try_convex_hull(points):
    try:
        hull = ConvexHull(points)
        return points[hull.vertices]
    except:
        if points.shape[0] <= 2:
            return points  # Trop peu de points : retourne tel quel
        if points.shape[0] >= MIN_SAMPLES:
            return bounding_box(points)  # Assez de points : approximation rectangulaire
        else:
            return get_wall_approx(points)  # Sinon, approximation linéaire

# Fonction principale de détection des obstacles
def detect_obstacles(plot=False):
    data = get_data()
    if not data:
        print("Aucune donnée reçue.")
        return []

    # Conversion polaire → cartésien
    angles = np.radians([p[0] for p in data])
    distances = np.array([p[1] for p in data])
    x = distances * np.cos(angles + np.pi / 2)
    y = distances * np.sin(angles + np.pi / 2)
    points = np.column_stack((x, y))

    # Suppression des points proches du LIDAR (souvent des parasites)
    points = points[np.linalg.norm(points, axis=1) > 20]

    # Clustering avec DBSCAN
    db = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(points)
    labels = db.labels_
    unique_labels = set(labels)

    obstacles = []
    obstacle_counter = 0

    # Si affichage graphique demandé
    if plot:
        plt.figure(figsize=(7,7))
        plt.scatter(x, y, color='blue', s=1, label="Points LIDAR")

    for label in unique_labels:
        if label == -1:
            continue  # -1 = bruit (points non classés)

        cluster = points[labels == label]
        if len(cluster) < MIN_SAMPLES:
            continue  # Cluster trop petit pour être considéré comme un obstacle

        # Si cluster trop grand, on le découpe avec KMeans
        if len(cluster) > MAX_POINTS:
            num_subclusters = int(np.ceil(len(cluster) / MAX_POINTS))
            kmeans = KMeans(n_clusters=num_subclusters, n_init=10).fit(cluster)
            sub_labels = kmeans.labels_

            for sub_label in set(sub_labels):
                subcluster = cluster[sub_labels == sub_label]
                if len(subcluster) < MIN_SAMPLES:
                    continue
                hull_points = try_convex_hull(subcluster)
                obstacles.append(Obstacle(f"Obstacle {obstacle_counter}", hull_points, subcluster))
                if plot:
                    # Dessin du contour ou approximation
                    if hull_points.shape[0] >= 3:
                        plt.fill(hull_points[:,0], hull_points[:,1], edgecolor='red', fill=False, linewidth=2)
                    elif hull_points.shape[0] == 2:
                        plt.plot(hull_points[:,0], hull_points[:,1], 'r-', linewidth=2)
                    else:
                        plt.plot(hull_points[0,0], hull_points[0,1], 'ro')
                    plt.text(*obstacles[-1].centroid, f"Obs {obstacle_counter}", fontsize=8, color='red', ha='center')
                obstacle_counter += 1
        else:
            # Cluster de taille raisonnable
            hull_points = try_convex_hull(cluster)
            obstacles.append(Obstacle(f"Obstacle {obstacle_counter}", hull_points, cluster))
            if plot:
                if hull_points.shape[0] >= 3:
                    plt.fill(hull_points[:,0], hull_points[:,1], edgecolor='red', fill=False, linewidth=2)
                elif hull_points.shape[0] == 2:
                    plt.plot(hull_points[:,0], hull_points[:,1], 'r-', linewidth=2)
                else:
                    plt.plot(hull_points[0,0], hull_points[0,1], 'ro')
                plt.text(*obstacles[-1].centroid, f"Obs {obstacle_counter}", fontsize=8, color='red', ha='center')
            obstacle_counter += 1

    if plot:
        # Mise en forme du graphique
        max_distance = 500
        plt.xlim(-max_distance, max_distance)
        plt.ylim(-max_distance, max_distance)
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.grid(True, linestyle='--', linewidth=0.5)
        plt.title("Obstacles détectés (nuage LIDAR)")
        plt.xlabel("x (mm)")
        plt.ylabel("y (mm)")
        plt.legend()
        plt.savefig("formes_detectees.png")
        plt.close()

    return obstacles

# Point d’entrée principal : lance la détection avec affichage graphique
if __name__ == "__main__":
    obs = detect_obstacles(plot=True)
    print(f"{len(obs)} obstacles détectés.")
