import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

# === RÉCEPTION DES DONNÉES LIDAR VIA SOCKET UDP ===
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
    # On crée une socket UDP côté serveur et on bind l'adresse localhost sur le port 8080
    server_socket.bind(('localhost', 8080))
    print("Waiting for server...")

    def get_data():
        measure = []
        # Réception initiale de données
        data = server_socket.recvfrom(1024)
        # Décodage des données reçues et extraction de l'angle et distance
        str_data = data[0].decode("utf-8")[3:].split(" ")
        angle = float(str_data[1])
        distance = float(str_data[3])
        start_time = time.time()

        # On ouvre un fichier pour sauvegarder la première mesure (angle et distance)
        with open("data.txt", "w") as doc_data:
            doc_data.write(f"{angle}    {distance}\n")
        
        # Pendant 3 secondes, on continue à recevoir des mesures et à les stocker dans la liste
        while time.time() - start_time < 3:
            data = server_socket.recvfrom(1024)
            str_data = data[0].decode("utf-8")[3:].split(" ")
            angle = float(str_data[1])
            distance = float(str_data[3])
            # On ajoute les mesures sous forme de tuple (-angle, distance) dans la liste
            measure.append((-angle, distance))
        return measure

    # On appelle la fonction pour récupérer les données pendant 3 secondes
    data = get_data()

# === CONVERSION DES COORDONNÉES POLAIRES EN CARTÉSIENNES ===
# Transformation des angles en radians (et inversion du signe de l'angle)
angles = np.radians([point[0] for point in data])
# Récupération des distances
distances = np.array([point[1] for point in data])
# Calcul des coordonnées x et y avec décalage de π/2 (90°)
x = distances * np.cos(angles + np.pi / 2)
y = distances * np.sin(angles + np.pi / 2)
# Création d’un tableau Nx2 des points (x,y)
points = np.column_stack((x, y))

# === CLUSTERING AVEC DBSCAN POUR DÉTECTER LES GROUPES DE POINTS (OBSTACLES) ===
# eps=20 : distance max entre points pour être dans un même cluster
# min_samples=10 : nombre minimal de points dans un cluster
db = DBSCAN(eps=20, min_samples=10).fit(points)
labels = db.labels_  # étiquette de cluster pour chaque point

# === TRAITEMENT POUR DESSINER LES FORMES ET IDENTIFIER LES OBSTACLES ===
unique_labels = set(labels)  # ensemble des clusters identifiés
obstacles = []  # liste pour stocker les obstacles détectés

plt.figure(figsize=(6, 6))  # création de la figure pour affichage
plt.scatter(x, y, color='blue', s=1, label="Points LiDAR")  # affichage des points bruts

for label in unique_labels:
    if label == -1:
        continue  # on ignore les points considérés comme bruit par DBSCAN

    cluster = points[labels == label]  # extraction des points du cluster
    if len(cluster) < 3:
        continue  # on ne peut pas calculer une enveloppe convexe avec moins de 3 points

    # === Calcul de l'enveloppe convexe du cluster ===
    try:
        hull = ConvexHull(cluster)
        hull_points = cluster[hull.vertices]

        # Affichage de l’enveloppe convexe (contour de l’obstacle)
        plt.fill(hull_points[:, 0], hull_points[:, 1], edgecolor='red', fill=False, linewidth=2)
        
        # On mémorise l'obstacle sous forme de dictionnaire (label et points du contour)
        obstacles.append({
            "label": f"Obstacle {label}",
            "points": hull_points.tolist()
        })

        # On place une annotation texte au centre (centroïde) de l’enveloppe convexe
        centroid = np.mean(hull_points, axis=0)
        plt.text(centroid[0], centroid[1], f"Obstacle {label}", fontsize=8, color='red', ha='center')
    except:
        # Si la computation de l'enveloppe échoue (ex: points colinéaires), on ignore
        pass

# === CONFIGURATION ET AFFICHAGE FINAL DU GRAPHIQUE ===
max_distance = 500  # limites pour axes x et y
plt.xlim(-max_distance, max_distance)
plt.ylim(-max_distance, max_distance)
plt.axhline(0, color='black', linewidth=0.5)  # axe horizontal
plt.axvline(0, color='black', linewidth=0.5)  # axe vertical
plt.grid(True, linestyle='--', linewidth=0.5)  # grille en pointillés
plt.title("Obstacles détectés avec le lidar")
plt.xlabel("x (mm)")
plt.ylabel("y (mm)")
plt.legend()
plt.savefig("formes_detectees.png")  # sauvegarde de la figure en image
plt.show()  # affichage de la figure

# === AFFICHAGE EN CONSOLE DES COORDONNÉES DES OBSTACLES ===
for ob in obstacles:
    print(ob["label"])
    for pt in ob["points"]:
        print(f"   Point: {pt}")
