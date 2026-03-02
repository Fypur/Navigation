from collections import deque
from typing import List, Tuple

Point = Tuple[float, float]

def dbscan_minimal(points: List[Point], eps: float, min_samples: int) -> List[int]:
    """
    Retourne la liste des points (x, y) appartenant au plus gros cluster (DBSCAN).

    Args:
        obs_coordinates liste de tuples (x, y)
        eps: distance max entre voisins (ici on a choisi 14,1 cm qui correspondent à sqrt(2)*plus grand côté de la balise)
        min_samples: nb min de points pour former un cluster

    Returns:
        liste de tuples (x, y) du plus gros cluster
        (liste vide si aucun cluster détecté)
    """

    n = len(points)
    if n == 0:
        return []

    eps2 = eps * eps  # compare au carré pour éviter sqrt
    labels = [-2] * n  # -2 = non visité, -1 = bruit, >=0 = cluster id

    def region_query(i: int) -> List[int]:
        """Renvoie les indices des points à distance <= eps du point i (inclut i)."""
        xi, yi = points[i]
        neigh = []
        for j, (xj, yj) in enumerate(points):
            dx = xj - xi
            dy = yj - yi
            if dx * dx + dy * dy <= eps2:
                neigh.append(j)
        return neigh

    cluster_id = 0

    for i in range(n):
        if labels[i] != -2:  # déjà visité
            continue

        neighbors = region_query(i)

        # Pas assez de voisins => bruit (pour l'instant)
        if len(neighbors) < min_samples:
            labels[i] = -1
            continue

        # Sinon on crée/étend un cluster
        labels[i] = cluster_id
        queue = deque(neighbors)

        while queue:
            j = queue.popleft()

            if labels[j] == -1:
                # était marqué bruit, mais finalement atteignable depuis un core -> devient border
                labels[j] = cluster_id

            if labels[j] != -2:
                # déjà assigné (bruit traité ci-dessus) ou déjà dans un cluster
                continue

            labels[j] = cluster_id
            j_neighbors = region_query(j)

            # Si j est un point "core", on ajoute ses voisins à explorer
            if len(j_neighbors) >= min_samples:
                queue.extend(j_neighbors)

        cluster_id += 1

    return labels


def largest_cluster(points: List[Point], eps, min_samples = 4) -> List[Point]:
    """
    Retourne la liste des (x,y) du plus gros cluster DBSCAN.
    Renvoie [] si aucun cluster.
    """
    labels = dbscan_minimal(points, eps, min_samples)

    # Compter les tailles de clusters (ignorer -1 et -2)
    counts = {}
    for lab in labels:
        if lab >= 0:
            counts[lab] = counts.get(lab, 0) + 1

    if not counts:
        return []

    largest_label = max(counts, key=counts.get)
    return [pt for pt, lab in zip(points, labels) if lab == largest_label]