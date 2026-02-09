import numpy as np
from scipy.spatial import KDTree  # Pour chercher les plus proches voisins efficacement

# --- Définition de la classe Obstacle ---
class Obstacle:
    """
    Représente un obstacle détecté, défini par :
    - un label (nom)
    - un contour (envelope)
    - les points du nuage LIDAR associés
    - un centroïde (moyenne des points)
    """
    def __init__(self, label, envelope, points):
        self.label = label
        self.envelope = envelope
        self.points = points
        self.centroid = np.mean(points, axis=0) if len(points) > 0 else np.array([0.0, 0.0])

    def __repr__(self):
        return f"{self.label} | Points: {len(self.points)} | Centroid: {self.centroid}"

# === FONCTIONS D'ALIGNEMENT ===

def find_rigid_transform(A, B):
    """
    Calcule la meilleure transformation rigide (rotation R et translation t)
    pour aligner deux ensembles de points A → B.
    Méthode utilisée : algorithme de SVD.
    """
    assert A.shape == B.shape  # A et B doivent avoir le même nombre de points

    # Centrer les ensembles autour de leurs centroïdes
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # Matrice de covariance
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)

    # Calcul de la rotation via SVD
    R = Vt.T @ U.T

    # Correction si la rotation obtenue est une réflexion (déterminant négatif)
    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = Vt.T @ U.T

    # Calcul de la translation associée
    t = centroid_B - R @ centroid_A
    return R, t

def match_closest_points(source, target, distance_threshold=100):
    """
    Associe chaque point de `source` à son plus proche voisin dans `target`
    à condition que la distance soit inférieure à `distance_threshold`.

    Retour :
    - matched_src : points de `source` associés
    - matched_tgt : points correspondants dans `target`
    """
    tree = KDTree(target)  # Accélère la recherche de plus proches voisins
    matched_src = []
    matched_tgt = []

    for pt in source:
        dist, idx = tree.query(pt)  # Recherche du point le plus proche dans `target`
        if dist < distance_threshold:
            matched_src.append(pt)
            matched_tgt.append(target[idx])

    return np.array(matched_src), np.array(matched_tgt)

# === ESTIMATION DU MOUVEMENT ===

def estimate_robot_motion(obstacles_1, obstacles_2):
    """
    Estime la transformation rigide (rotation + translation)
    entre deux instants (t1 et t2) à partir des centroïdes des obstacles.

    Étapes :
    1. Extraire les centroïdes de chaque obstacle
    2. Trouver les paires correspondantes (obstacles proches)
    3. Calculer la rotation + translation qui aligne t1 → t2
    """
    centroids_1 = np.array([ob.centroid for ob in obstacles_1])
    centroids_2 = np.array([ob.centroid for ob in obstacles_2])

    # Associer les centroïdes les plus proches entre t1 et t2
    matched_1, matched_2 = match_closest_points(centroids_1, centroids_2)

    # Vérifier qu'on a assez de correspondances pour calculer une transformation
    if len(matched_1) < 2:
        print("Pas assez de correspondances fiables pour estimer un mouvement.")
        return None

    # Calcul de la transformation rigide
    R, t = find_rigid_transform(matched_1, matched_2)

    # Affichage des résultats
    print("\n=== TRANSFORMATION ESTIMÉE ===")
    print("Rotation :\n", R)
    print("Translation :", t)

    return R, t
