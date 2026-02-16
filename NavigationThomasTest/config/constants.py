# --- CONFIGURATION SÉRIE (Arduino) ---
BAUDRATE = 115200
SERIAL_PORT = '/dev/ttyACM0' 

# --- CONFIGURATION MQTT ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

TOPIC_MODE = "robot/control/mode"
TOPIC_MOTOR_CMD = "robot/actuators/cmd_vel"
TOPIC_LIDAR = "robot/sensors/lidar"
TOPIC_MANUAL_CMD = "robot/control/manual_cmd"
TOPIC_ODOM = "robot/sensors/odom"

# --- CONFIGURATION PHYSIQUE (À MESURER ) ---
# Rayon de la roue en mètres (ex: 3cm = 0.03)
WHEEL_RADIUS = 0.03  
# Distance entre les roues gauche et droite en mètres (voie)
ROBOT_WIDTH = 0.20   
# Nombre de ticks pour faire un tour complet de roue
# doc des moteurs ou test (tourner 1 tour et regarder le compteur)
TICKS_PER_REV = 600  

# Calcul automatique des ratios
METERS_PER_TICK = (2 * 3.14159 * WHEEL_RADIUS) / TICKS_PER_REV

# --- PINOUT GPIO (BCM) ---
# Format: (CLK, DT)
ENC_M1 = (17, 18) # Avant Gauche ?
ENC_M2 = (22, 23) # Avant Droit ?
ENC_M3 = (20, 26) # Arrière Gauche ? (verifier si c'est 20/26 ou 12/6 sur ta carte)
ENC_M4 = (4, 14)  # Arrière Droit ?
