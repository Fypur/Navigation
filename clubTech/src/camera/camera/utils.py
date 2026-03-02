import cv2
import numpy as np
from game_parameters import *

# Utils
def open_camera(resolution=default_resolution, camera_index=camera_index):
    """Open the camera and set the resolution."""
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print("Error: could not open camera.")
        return None
    
    # Set camera codec to MJPG to improve performance
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    
    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    
    return cap


def get_corners(L): # L is the length of a marker side
    """Returns the corners of a marker in its referential (origin = marker's center)"""
    half = L / 2.0
    
    return np.array([
        [-half, half, 0.0], # Top Left
        [half, half, 0.0], # Top Right
        [half, -half, 0.0], # Bottom Right
        [-half, -half, 0.0] # Bottom Left
    ])


def get_corners_in_world(center_world, L):
    """Returns the corners of the marker in the world referential"""
    return get_corners(L) + center_world.reshape(1,3)


def load_camera_calibration(file=calibration_file):
    """Returns the precomputed camera parameters: K and dist"""
    
    try:
        data = np.load(file)

        K = data["K"]
        dist = data["dist"]
    
        return K, dist
    
    except:
        raise FileNotFoundError("Precomputed camera parameters not found. Please run the camera calibration procedure.")
    

def get_camera_position(capture=None, n_obs=50, obs_limit=250, calibration=None):
    """
    Return camera position estimated from detected reference markers.
    """
    
    # 1. Gestion des listes pour la moyenne (On stocke les vecteurs, pas les matrices)
    rvec_list = [] 
    t_cw_list = []
    
    # 2. Gestion de la caméra (Ouverture interne ou utilisation de l'objet passé)
    opened_internally = False
    if capture is None:
        cap = cv2.VideoCapture(0) # ou camera_index si défini globalement
        # Configuration résolution si besoin
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        opened_internally = True
    else:
        cap = capture
        
    # Vérification
    if not cap.isOpened():
        if opened_internally:
            cap.release()
        print("Error: could not open camera.")
        return False, None, None

    # 3. Récupération de la calibration
    if calibration is None:
        # Assurez-vous que load_camera_calibration() est accessible ou passez les args
        # K, dist = load_camera_calibration() 
        print("Error: Calibration data missing.")
        if opened_internally: cap.release()
        return False, None, None
    else:
        K, dist = calibration
    
    # --- BOUCLE D'ESTIMATION ---
    i, s = 0, 0
    try:
        while i < obs_limit and len(rvec_list) < n_obs:
            i += 1
            ret, frame = cap.read()
            if not ret: continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Utilisation des variables globales ARUCO_DICT et aruco_type
            aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            
            corners_list, marker_ids, _ = detector.detectMarkers(gray)

            if marker_ids is None or len(marker_ids) == 0: continue

            obj_points = []
            img_points = []
            marker_ids_flat = marker_ids.flatten()

            for corner, marker_id in zip(corners_list, marker_ids_flat):
                marker_id = int(marker_id)
                
                # Utilisation de la variable globale reference_ids
                if marker_id not in reference_ids:
                    continue

                img_point = corner.reshape(4, 2).astype(np.float64)
                
                # Utilisation des globales reference_markers et reference_marker_size
                obj_point = get_corners_in_world(
                    reference_markers[marker_id],
                    reference_marker_size
                )
                
                obj_points.append(obj_point)
                img_points.append(img_point)

            if len(obj_points) == 0: continue

            # Formatage pour solvePnP
            obj_points_np = np.vstack(obj_points)
            img_points_np = np.vstack(img_points)

            ok, rvec, tvec = cv2.solvePnP(
                obj_points_np, img_points_np, K, dist,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not ok: continue

            # CORRECTION CRITIQUE : On stocke rvec (vecteur), pas R (matrice)
            rvec_list.append(rvec)
            t_cw_list.append(tvec)
            
            s += 1
            if s % 10 == 0:
                print(f"{s} successful observations collected out of {i}.")

        # --- FIN DE BOUCLE ---
        
        if len(rvec_list) < n_obs:
            print("Camera pose estimation was unsuccessful (not enough data).")
            return False, None, None

        # Moyenne des positions
        # On peut faire la moyenne des rvecs car ce sont des vecteurs de rotation (axis-angle)
        # Pour des petites variations (caméra fixe), l'approximation est valide.
        rvec_array = np.stack(rvec_list, axis=0) # (N, 3, 1)
        t_cw_array = np.stack(t_cw_list, axis=0) # (N, 3, 1)

        rvec_mean = np.mean(rvec_array, axis=0)
        t_cw_mean = np.mean(t_cw_array, axis=0)

        # Conversion finale en Matrice de Rotation
        R_cw_mean, _ = cv2.Rodrigues(rvec_mean)

        return True, R_cw_mean, t_cw_mean

    finally:
        # On ne ferme la caméra que si c'est la fonction qui l'a ouverte
        if opened_internally:
            cap.release()


def project_on_table_plane(center_world, R_cw, t_cw, height=0.0):
    """Project a 3D world point onto the horizontal plane z = height
    along the line of sight from the camera.

    Args:
        center_world : np.ndarray (3,) or (3,1)
            3D point in world coordinates.
        R_cw : (3,3)
            Rotation matrix from world to camera.
        t_cw : (3,1)
            Translation vector from world to camera.
        height : float
            Height (z-coordinate) of the plane onto which the point is projected.

    Returns:
        np.ndarray (3,)
            The projected point on the plane z = height.
    """

    center_world = center_world.reshape(3, 1)

    # Camera optical center in world coordinates:
    C_w = -R_cw.T @ t_cw

    Cz = C_w[2, 0]
    Pz = center_world[2, 0]

    # If the ray is parallel to the plane: no meaningful intersection
    if abs(Pz - Cz) < 1e-9:
        return center_world.flatten()

    # Solve for λ such that Z(λ) = height :
    # C_z + λ (P_z - C_z) = height  => λ = (height - C_z) / (P_z - C_z)
    lam = (height - Cz) / (Pz - Cz)

    Q_w = C_w + lam * (center_world - C_w)

    return Q_w.flatten()



def find_marker_id(capture = None, marker_ids = reference_ids, marker_size = reference_marker_size, camera_pose = None, calibration = None, height = 0.0):
    """Find the position of a given marker ID in the world referential.

    Args:
        capture (cv2.VideoCapture): Camera capture object
        marker_ids (set): Set of marker IDs to search for
        marker_size (float): Size of the marker side in meters
        camera_pose (tuple): Tuple (R_cw, t_cw) of the camera pose
        calibration (tuple): Tuple (K, dist) of the camera calibration parameters
        
    Returns:
        positions_world (dict): Dictionary mapping marker IDs to their 3D world positions
    """
    
    # Open the default (int = 0) camera
    if capture is None:
        cap = open_camera()
        
    else:
        cap = capture
    
    # Import calibration
    if calibration is None:
        calibration = load_camera_calibration()
    K, dist = calibration
    
    # Find camera pose if not provided
    if camera_pose is None:
        ok_cam, R_cw, t_cw = get_camera_position(capture=cap, n_obs=1, obs_limit=obs_limit, calibration=calibration)
        
        if not ok_cam:
            print("Marker position estimation failed. Cause: unable to estimate camera position.")
            return None
        
    else:
        _, R_cw, t_cw = camera_pose
    
    # Reading the image
    ret, frame = cap.read()  # ret is a bool indicating if the image was read successfully
    
    if not ret:
        print("Marker position estimation failed. Cause: failed to grab frame")
        return None
    
    # Find ArUco markers in the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners_list, marker_ids_detected, _ = detector.detectMarkers(gray)
    
    # Handle the case where no markers are detected
    if marker_ids_detected is None or len(marker_ids_detected) == 0:
        print("Marker position estimation failed. Cause: no marker detected in the frame.")
        return None
    
    # Estimate markers position
    positions_world = {}
    orientations_world = {}
    
    # 3D coordinates of the corners of a marker in its own referential
    object_point_centered  = get_corners(marker_size)
    
    # For each detected marker, estimate its position in world coordinates
    for corner, marker_id in zip(corners_list, marker_ids_detected.flatten()):
        # Extracting the marker ID
        marker_id = int(marker_id)
        
        # Consider only the requested marker IDs
        if marker_id not in marker_ids:
            continue
        
        # Reshaping the corner into a 4x2 array
        image_point = corner.reshape(4,2).astype(np.float64)
    
        # Choosing the PnP solving method
        # Use IPPE for square markers if available, else fallback to ITERATIVE        
        flag = getattr(cv2,"SOLVEPNP_IPPE_SQUARE",cv2.SOLVEPNP_ITERATIVE)
        
        # Solve the marker pose relative to the camera
        ok_marker, rvec_marker, tvec_marker = cv2.solvePnP(
            object_point_centered,
            image_point,
            K,
            dist,
            flags=flag)
        
        # If PnP failed for this marker, skip it        
        if not ok_marker:
            continue
        
        # Rotation matrix from marker to camera
        R_cm, _ = cv2.Rodrigues(rvec_marker)
        
        # Translation vector from marker to camera
        t_cm = tvec_marker
        
        # Converting the maker center from camera to world coordinates
        # X_world = R_cw^T (X_cam - t_cw)
        center_world = R_cw.T @ (tvec_marker - t_cw)
        center_world = project_on_table_plane(center_world, R_cw, t_cw, height= height)
        
        positions_world[marker_id] = center_world

        # Orientation of the marker in world coordinates
        R_mw = R_cw.T @ R_cm
        theta = np.arctan2(R_mw[1, 0], R_mw[0, 0])
        orientations_world[marker_id] = theta
    
    if capture is None:
        cap.release()
    
    return positions_world, orientations_world


def find_bms(capture = None, camera_pose = None, calibration = None, IDs = None, marker_size = bm_marker_size, height = bm_height):
    """Find the positions and orientations of BM markers in the world frame.

    Args:
        capture: Optional cv2.VideoCapture instance. If None, a new camera is opened.
        camera_pose: Optional tuple (ok_cam, R_cw, t_cw). If None, the camera pose
            is estimated using get_camera_position(...).
        calibration: Optional tuple (K, dist). If None, load_camera_calibration()
            is called.
        IDs: Set of marker IDs to look for.
        marker_size: Physical size of the marker (edge length, in meters).

    Returns:
        positions_world: dict[int, list[np.ndarray]], mapping marker ID → list of 3D positions.
        orientations_world: dict[int, list[float]], mapping marker ID → list of yaw angles (rad).
        Returns None in case of failure (no camera pose, no frame, no markers, etc.).
    """

    if IDs is None:
        IDs = {blue_id, yellow_id, empty_id}
    
    # Open the default (int = 0) camera
    if capture is None:
        cap = open_camera()
        
    else:
        cap = capture
    
    # Import calibration
    if calibration is None:
        calibration = load_camera_calibration()
    K, dist = calibration
    
    # Find camera pose if not provided
    if camera_pose is None:
        ok_cam, R_cw, t_cw = get_camera_position(capture=cap, n_obs=1, obs_limit=obs_limit, calibration=calibration)
        
        if not ok_cam:
            print("Marker position estimation failed. Cause: unable to estimate camera position.")
            return None
        
    else:
        _, R_cw, t_cw = camera_pose
    
    # Reading the image
    ret, frame = cap.read()  # ret is a bool indicating if the image was
    
    if not ret:
        print("Marker position estimation failed. Cause: failed to grab frame")
        return None
    
    # Find ArUco markers in the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners_list, marker_ids_detected, _ = detector.detectMarkers(gray)
    
    # Handle the case where no markers are detected
    if marker_ids_detected is None or len(marker_ids_detected) == 0:
        print("Marker position estimation failed. Cause: no marker detected in the frame.")
        return None
    
    # Estimate markers position
    positions_world = {marker_id : [] for marker_id in IDs}
    orientations_world = {marker_id : [] for marker_id in IDs}
    
    # 3D coordinates of the corners of a marker in its own referential
    object_point_centered  = get_corners(marker_size)
    
    # For each detected marker, estimate its position in world coordinates
    for corner, marker_id in zip(corners_list, marker_ids_detected.flatten()):
        # Extracting the marker ID
        marker_id = int(marker_id)
        
        # Consider only the requested marker IDs
        if marker_id not in IDs:
            continue
        
        # Reshaping the corner into a 4x2 array
        image_point = corner.reshape(4,2).astype(np.float64)
    
        # Choosing the PnP solving method
        # Use IPPE for square markers if available, else fallback to ITERATIVE        
        flag = getattr(cv2,"SOLVEPNP_IPPE_SQUARE",cv2.SOLVEPNP_ITERATIVE)
        
        # Solve the marker pose relative to the camera
        ok_marker, rvec_marker, tvec_marker = cv2.solvePnP(
            object_point_centered,
            image_point,
            K,
            dist,
            flags=flag)
        
        # If PnP failed for this marker, skip it        
        if not ok_marker:
            continue
        
        # Rotation matrix from marker to camera
        R_cm, _ = cv2.Rodrigues(rvec_marker)
        
        # Translation vector from marker to camera
        t_cm = tvec_marker
        
        # Converting the maker center from camera to world coordinates
        # X_world = R_cw^T (X_cam - t_cw)
        center_world = R_cw.T @ (tvec_marker - t_cw)
        center_world = project_on_table_plane(center_world, R_cw, t_cw, height = height)
        
        positions_world[marker_id].append(center_world)

        # Orientation of the marker in world coordinates
        R_mw = R_cw.T @ R_cm
        theta = np.arctan2(R_mw[1, 0], R_mw[0, 0])
        orientations_world[marker_id].append(theta)
    
    if capture is None:
        cap.release()
    
    return positions_world, orientations_world