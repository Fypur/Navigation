import cv2
import numpy as np

# Defining ArUCo marker type
ARUCO_DICT = {
    "DICT_4X4_100" : cv2.aruco.DICT_4X4_100,
}

# Camera loop
aruco_type = "DICT_4X4_100"

# Robot marker parameters
blue_robot_ids = list(range(1,6)) # possible IDs for blue robot
yellow_robot_ids = list(range(6,11)) # possible IDs for yellow robot
robot_marker_size = 0.07 # Side of a robot marker
robot_height = 0.43 # Height of the robot marker on the robot

# Table markers parameters
reference_markers = {
    # ID : np.array([x,y,z])
    20: np.array([0.600, 1.400, 0.0]),
    21: np.array([2.400, 1.400, 0.0]),
    22: np.array([0.600, 0.600, 0.0]),
    23: np.array([2.400, 0.600, 0.0]),
}

reference_ids = set(reference_markers.keys()) # Saving the reference marker IDs as an independent list
reference_marker_size = 0.100 # Side of a reference marker

# BM markers parameters
blue_id = 36 # ID of the blue BM marker
yellow_id = 47 # ID of the yellow BM marker
empty_id = 41 # ID of the empty BM marker
bm_marker_size = 0.04 # Side of a BM marker
bm_height = 0.03 # Height of the BM on the table

n_obs = 50 # Number of observation in camera position estimation 
obs_limit = 250 # Maximum number of tries to estimate camera position

camera_index = 1 # Index of the camera to use
default_resolution = (1280,720) # Default camera resolution
calibration_file = "CT_camera_calibration.npz" # Camera calibration file