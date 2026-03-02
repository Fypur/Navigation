"""
Ce fichier définit les paramètres utilisés par le node IMU.
"""

BAUD = 3000000
PORT = '/dev/ttyAMA0'
RESET_GPIO_CHIP = 4
RESET_GPIO_LINE = 4
topic = "/data_imu"
timer_period = 0.001
calibration_samples = 50
calibration_iterations_limit = 250
initial_yaw = 0.0 # Angle initial, en degrés