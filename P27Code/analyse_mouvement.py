import time
import matplotlib.pyplot as plt
from detection_obstacle import detect_obstacles
from matrice_rotation___vecteur_translation import estimate_robot_motion

# Variables globales pour stocker les obstacles à deux instants différents
obstacles_1 = []
obstacles_2 = []

# Fonction d'affichage des obstacles détectés
def plot_obstacles(obstacles, title="Obstacles détectés"):
    plt.ion()  # Active le mode interactif (mise à jour temps réel)
    plt.figure(figsize=(8,8))  # Création de la figure

    # Affichage des points LIDAR de chaque obstacle (en bleu)
    for ob in obstacles:
        pts = ob.points
        plt.scatter(pts[:,0], pts[:,1], s=5, c='blue', alpha=0.5)

    # Affichage de l’enveloppe de chaque obstacle (en rouge)
    for ob in obstacles:
        hull = ob.envelope
        if hull.shape[0] >= 3:
            plt.fill(hull[:,0], hull[:,1], edgecolor='red', fill=False, linewidth=2)
        elif hull.shape[0] == 2:
            plt.plot(hull[:,0], hull[:,1], 'r-', linewidth=2)
        else:
            plt.plot(hull[0,0], hull[0,1], 'ro')  # Point isolé

    # Affichage des centroïdes (en vert) avec étiquette
    for ob in obstacles:
        plt.plot(ob.centroid[0], ob.centroid[1], 'go')
        plt.text(ob.centroid[0], ob.centroid[1], ob.label, fontsize=8, color='green', ha='center')

    # Axes, grille et échelle
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.grid(True, linestyle='--', linewidth=0.5)

    plt.title(title)
    plt.xlabel("x (mm)")
    plt.ylabel("y (mm)")
    plt.axis('equal')  # Garde le même ratio x/y
    plt.xlim(-300, 300)
    plt.ylim(-300, 300)
    plt.draw()
    plt.pause(0.001)  # Laisse le temps d'afficher avant de continuer

# Fonction principale qui effectue deux captures et analyse le déplacement du robot
def analyse_mouvement():
    global obstacles_1, obstacles_2

    print("Capture 1 : scanner l'environnement...")
    obstacles_1 = detect_obstacles(plot=False)  # Première détection
    plot_obstacles(obstacles_1, title="Obstacles t1")  # Affichage

    input("Appuie sur Entrée pour lancer la capture 2...")  # Pause utilisateur

    print("Capture 2 : scanner l'environnement...")
    obstacles_2 = detect_obstacles(plot=False)  # Deuxième détection
    plot_obstacles(obstacles_2, title="Obstacles t2")  # Affichage

    print("\nCalcul de la transformation du robot entre t1 et t2...")
    estimate_robot_motion(obstacles_1, obstacles_2)  # Estimation du mouvement (rotation + translation)

    input("Appuie sur Entrée pour fermer les figures...")
    plt.close('all')  # Ferme toutes les fenêtres matplotlib

# Point d'entrée du script
if __name__ == "__main__":
    analyse_mouvement()
