import sys
import os

# Configuration de la détection
SAFE_DISTANCE = 500.0  # Distance de sécurité en mm (50 cm)
CRITICAL_DISTANCE = 200.0 # Arrêt d'urgence (20 cm)
SECTOR_ANGLE = 30.0    # Angle de vue (30° gauche, 30° droite)

def get_autonomous_command(scan_data):
    """
    Analyse les données LiDAR et renvoie une commande de vitesse.
    scan_data : Liste de dicts [{"a": angle, "d": distance}, ...]
    """
    
    # Initialisation des distances minimales par secteur
    # On met une grande valeur par défaut
    min_dist_left = 10000
    min_dist_center = 10000
    min_dist_right = 10000

    # 1. On trie les points dans les secteurs (Gauche, Centre, Droite)
    # On suppose que l'angle 0° est devant. 
    # On normalise : 
    #   0 à 30° -> Gauche (ou Droite)
    #   330 à 360° -> Droite (ou Gauche)
    
    for point in scan_data:
        angle = point['a']
        dist = point['d']

        # Filtre bruit (points trop proches ou nuls)
        if dist < 10: 
            continue

        # SECTEUR CENTRE (Devant : -15° à +15° soit 345-360 et 0-15)
        if angle < 15 or angle > 345:
            if dist < min_dist_center:
                min_dist_center = dist
        
        # SECTEUR GAUCHE (15° à 45°)
        elif 15 <= angle < 45:
            if dist < min_dist_left:
                min_dist_left = dist
                
        # SECTEUR DROITE (315° à 345°)
        elif 315 < angle <= 345:
            if dist < min_dist_right:
                min_dist_right = dist

    # 2. Prise de décision
    
    # Cas A : MUR EN FACE TRÈS PROCHE (Arrêt urgence)
    if min_dist_center < CRITICAL_DISTANCE:
        print("OBSTACLE CRITIQUE")
        return {"linear": 0, "angular": 0}

    # Cas B : OBSTACLE DEVANT (On tourne)
    if min_dist_center < SAFE_DISTANCE:
        # On choisit le côté le plus libre
        if min_dist_left > min_dist_right:
            # Gauche est plus libre -> On tourne à gauche
            return {"linear": 0, "m1": -30, "m2": 30, "m3": -30, "m4": 30} # Rotation sur place
        else:
            # Droite est plus libre -> On tourne à droite
            return {"linear": 0, "m1": 30, "m2": -30, "m3": 30, "m4": -30}

    # Cas C : OBSTACLE SUR LE CÔTÉ (On corrige légèrement)
    if min_dist_left < SAFE_DISTANCE:
        # Trop près à gauche -> on vire un peu à droite
        return {"linear": 20, "angular": -10} # Avance + Tourne Droite
        
    if min_dist_right < SAFE_DISTANCE:
        # Trop près à droite -> on vire un peu à gauche
        return {"linear": 20, "angular": 10} # Avance + Tourne Gauche

    # Cas D : VOIE LIBRE
    return {"linear": 40} # Avance tout droit (vitesse modérée)