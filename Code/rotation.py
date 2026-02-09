import socket
import time
import numpy as np
import math
import matplotlib.pyplot as plt

"You can test this file by placing the Lidar, you lunch  point_A=recuperer_centroid() in the Python shell, you rotate the lidar WITHOUT TRANSLATE IT"
"you lunch again point_B=recuperer_centroid(), and you calculate_rotation_angle(point_A, point_B)"

def recuperer_centroid(distance_filter=1000, duration=3): #cet algorithme permet de cartographier l'espace, de prendre le centroide (la moyenne des points) de cette carte
    "This algorithm allows you to map the space, taking the centroid (the average of the points) of this map."
    "You can change the measurement distance (in m) and the acquisition time (number of points and precision)"
    points = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
        server_socket.bind(('localhost', 8080))
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                data, _ = server_socket.recvfrom(1024)
                str_data = data.decode("utf-8")[3:].split(" ")
                angle = float(str_data[1])  # Assurez-vous que l'angle est dans le bon format
                distance = float(str_data[3])
                if distance > 0:  # Filtrer les distances non positives
                    points.append((angle, distance))  # Ne pas inverser l'angle ici
            except Exception as e:
                print(f"Erreur dans la réception des données: {e}")
                continue

    if not points:
        return np.array([0.0, 0.0])

    # Filtrer les points selon la distance
    filtered_points = [p for p in points if p[1] < distance_filter]

    if len(filtered_points) == 0:
        return np.array([0.0, 0.0])

    # Calcul de la moyenne des angles en prenant en compte la continuité des angles
    angles = np.array([p[0] for p in filtered_points])
    distances = np.array([p[1] for p in filtered_points])

    # Gestion des angles avec un wraparound
    sin_mean = np.mean(np.sin(np.radians(angles)))
    cos_mean = np.mean(np.cos(np.radians(angles)))
    mean_angle = np.degrees(np.arctan2(sin_mean, cos_mean))

    # Moyenne des distances
    mean_distance = np.mean(distances)

    centroid = np.array([mean_angle, mean_distance])

    return centroid

def calculate_rotation_angle(centroid_1, centroid_2):
    "This algorithm allows you to calculate the angle between two centroids"
    # Calcul de la différence angulaire en utilisant les coordonnées polaires
    angle_diff = centroid_1[0] - centroid_2[0]
    # S'assurer que l'angle reste dans l'intervalle [-180°, 180°]
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360
    
    return angle_diff
