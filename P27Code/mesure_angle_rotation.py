import socket
import time
import numpy as np
import matplotlib.pyplot as plt

# === RÉCEPTION DONNÉES LIDAR ===
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
    server_socket.bind(('localhost', 8080))
    print("Attente du serveur...")

    def get_data():
        measure = []
        data = server_socket.recvfrom(1024)
        str_data = data[0].decode("utf-8")[3:].split(" ")
        angle = float(str_data[1])
        distance = float(str_data[3])
        start_time = time.time()
        while time.time() - start_time < 5:
            data = server_socket.recvfrom(1024)
            str_data = data[0].decode("utf-8")[3:].split(" ")
            angle = float(str_data[1])
            distance = float(str_data[3])
            measure.append((-angle, distance))
        return measure

    # Capture de données initiales (t=0)
    print("Capture de données à t=0...")
    data_1 = get_data()
    print("tourner")
    time.sleep(8)  # Attendre 5 secondes avant de prendre une autre mesure

    # Capture de données après une rotation (t=5s)
    print("Capture de données à t=5s...")
    data_2 = get_data()

# === CONVERSION POLAIRE → CARTÉSIENNE ===
def polar_to_cartesian(data):
    angles = np.radians([point[0] for point in data])
    distances = np.array([point[1] for point in data])
    x = distances * np.cos(angles + np.pi / 2)
    y = distances * np.sin(angles + np.pi / 2)
    return np.column_stack((x, y))

points_1 = polar_to_cartesian(data_1)
points_2 = polar_to_cartesian(data_2)

# === FILTRAGE DES POINTS À MOINS DE 50 CM ===
def filter_points(points, threshold_distance=400):  # 50 cm = 500 mm
    # Calculer la distance euclidienne pour chaque point par rapport à l'origine (0, 0)
    distances = np.linalg.norm(points, axis=1)
    # Filtrer les points dont la distance est inférieure au seuil
    return points[distances < threshold_distance]

points_1_filtered = filter_points(points_1)
points_2_filtered = filter_points(points_2)

# === CALCULER LE CENTROÏDE DE CHAQUE SCAN ===
def calculate_centroid(points):
    return np.mean(points, axis=0)

centroid_1 = calculate_centroid(points_1_filtered)
centroid_2 = calculate_centroid(points_2_filtered)

# === CALCULER L'ANGLE DE ROTATION À PARTIR DE L'ALIGNEMENT DES CENTROÏDES ===
def calculate_rotation_angle(centroid_1, centroid_2):
    # Calcul de la différence angulaire
    theta_1 = np.arctan2(centroid_1[1], centroid_1[0])
    theta_2 = np.arctan2(centroid_2[1], centroid_2[0])
    angle1 = np.degrees(theta_2-theta_1)
    
    return angle1

rotation_angle = calculate_rotation_angle(centroid_1, centroid_2)

print(f"L'angle de rotation estimé est de {rotation_angle:.2f} degrés.")

# === AFFICHAGE DES SCANS AVANT ET APRÈS LA ROTATION ===
plt.figure(figsize=(6, 6))

# Afficher les deux scans filtrés
plt.scatter(points_1_filtered[:, 0], points_1_filtered[:, 1], color='blue', s=1, label="Scan à t=0")
plt.scatter(points_2_filtered[:, 0], points_2_filtered[:, 1], color='red', s=1, label="Scan à t=5s")

# Afficher les centroïdes
plt.scatter(centroid_1[0], centroid_1[1], color='blue', marker='x', label="Centroïde t=0")
plt.scatter(centroid_2[0], centroid_2[1], color='red', marker='x', label="Centroïde t=5s")

plt.title("Scans avant et après la rotation")
plt.xlabel("x (mm)")
plt.ylabel("y (mm)")
plt.legend()
plt.grid(True)
plt.show()
