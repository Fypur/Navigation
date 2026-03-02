"""
Fichier pour définir les constantes et paramètres relatifs aux nodes sensor_bridge et offset. 
"""
from numpy import pi

# Cet offset est ajouté à la position renvoyée par l'EKF pour donner la position du robot dans le repère de la table.
# L'angle yaw est en radians
initial_offset = {'x' : 0.0, 'y' : 0.0, 'yaw' : pi/2} 

ekf_output_topic_name = '/odometry/filtered'
offset_output_topic_name = '/pos'